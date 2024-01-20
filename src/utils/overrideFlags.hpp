#ifndef OVERRIDE_FLAGS_H_
#define OVERRIDE_FLAGS_H_

namespace Utils{
    enum OverrideFlags{
        NONE = 1 >> 1,
        OVERRIDE = 1 << 0,
        OVERRIDEALL = 1 << 1
    };
    static bool inOverrideFlags(int val){
        return (
            (val & OverrideFlags::NONE) |
            (val & OverrideFlags::OVERRIDE) |
            (val & OverrideFlags::OVERRIDEALL)
        );
    }
}

#endif