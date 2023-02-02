#ifndef OPTION_H
#define OPTION_H

#define match(a, b) a.val; if(!a.has) {return Option<b>();}

template <typename T>
struct Option {
    T val;
    bool has;

    Option() : has(false) {}
    Option(const T& value) : has(true), val(value) {}
};

#endif