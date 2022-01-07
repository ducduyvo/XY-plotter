#ifndef GCODE_H_
#define GCODE_H_

#include <string>
#include <array>

#define OK_MESSAGE "OK\r\n"

// Create unique 16 bit ID char letter being the 8 MSB and number 8 LSB
// Type cast so it's easier to use it in code
#define CREATE_GCODE_ID(letter, number) ((Gcode::Id)(((letter << 8) | (number)) & 0xFFFF))
#define GET_LETTER_FROM_ID(id) ((id >> 8) & 0xFF)
#define GET_NUMBER_FROM_ID(id) ((id)      & 0xFF)

class Gcode {
public:
    using array = std::array<char, 4>;
    //Letter and number enums are there just to limit options for correct gcodes
    // The class could handle all combinations of letters and numbers 0-255
    enum Letter : char { M = 'M', G = 'G' };
    enum Number : uint8_t { _1 = 1, _2 = 2, _4 = 4, _5 = 5, _10 = 10, _11 = 11, _28 = 28 };
    enum Id : uint16_t {
        G1  = CREATE_GCODE_ID(Letter::G, Number::_1),
        G28 = CREATE_GCODE_ID(Letter::G, Number::_28),
        M1  = CREATE_GCODE_ID(Letter::M, Number::_1),
        M2  = CREATE_GCODE_ID(Letter::M, Number::_2),
        M4  = CREATE_GCODE_ID(Letter::M, Number::_4),
        M5  = CREATE_GCODE_ID(Letter::M, Number::_5),
        M10 = CREATE_GCODE_ID(Letter::M, Number::_10),
        M11 = CREATE_GCODE_ID(Letter::M, Number::_11)
    };

    // Union used for holding actual data since one of the structs can be used at a time
    // This will greatly reduce the Freertos Queue size
    typedef struct Data {
        Id id;
        union data {
            struct m1  { uint8_t penPos; } m1;
            struct m2  { uint8_t savePenUp; uint8_t savePenDown; } m2;
            struct m4  { uint8_t laserPower; } m4;
            struct m5  {
            	// if changing the alignment so speed is after dirY for better memory alignment it cant get width data with sscanf
                bool dirX ;
                bool dirY ;
                uint32_t height ;
                uint32_t width ;
                uint8_t speed ;
            } m5;
            struct g1  { // g1 and g28 have same data
                float moveX ;
                float moveY ;
                bool relative;
            } g1;
        } data;
    } Data;


    Gcode(Letter letter, Number number, bool (*functionPtr_)(const char *str) = nullptr)  :
        letter(letter),
        number(number),
        id(CREATE_GCODE_ID(letter, number)),
        functionPtr(functionPtr_)
    { }
    static const char* toFormat(const Id& id);
    static array toArray(const Letter& letter, const Number& number);
    static array toArray(const Id& id);
    const char* toFormat() { return toFormat(id); }
    array toString() { return toArray(letter, number); }
    Id getId() { return id; };
    bool callback(const char *str);
    virtual ~Gcode() { };

private:
    Letter letter;
    Number number;
    Id id;
    bool (*functionPtr)(const char *str);
};

#endif /* GCODE_H_ */
