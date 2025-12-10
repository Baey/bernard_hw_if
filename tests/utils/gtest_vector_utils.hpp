// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <gmock/gmock.h>

#define EXPECT_CALL_ALL(vec, mock_type, method_call)                     \
    do {                                                                 \
        for (auto& m : vec) {                                            \
            EXPECT_CALL(*static_cast<mock_type*>(m.get()), method_call); \
        }                                                                \
    } while (0)

#define EXPECT_CALL_ALL_TIMES(vec, mock_type, method_call, times)                     \
    do {                                                                              \
        for (auto& m : vec) {                                                         \
            EXPECT_CALL(*static_cast<mock_type*>(m.get()), method_call).Times(times); \
        }                                                                             \
    } while (0)
