#pragma once

#include <stdint.h>

enum class TofBusState : uint8_t
{
    Ready = 0,
    LowPending,
    SdaStuck,
    SclBlocked,
    Recovering
};
