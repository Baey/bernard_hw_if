// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

#include "I_communication_interface.hpp"

/// @brief MockResponder class to simulate device responses for testing
class MockResponder {
   public:
    using Predicate = std::function<bool(const std::vector<uint8_t>&)>;

    /// @brief Construct a new Mock Responder object
    MockResponder();

    /// @brief Destroy the Mock Responder object
    ~MockResponder();

    /// @brief Add a fixed response for requests starting with a given prefix
    void addFixedResponse(const std::vector<uint8_t>& requestPrefix,
                          const std::vector<uint8_t>& response,
                          mab::I_CommunicationInterface::Error_t err = mab::I_CommunicationInterface::OK);

    /// @brief Add a response for requests matching a given predicate
    void addPredicateResponse(Predicate pred,
                              const std::vector<uint8_t>& response,
                              mab::I_CommunicationInterface::Error_t err = mab::I_CommunicationInterface::OK);

    /// @brief Handle a transfer request and provide a response
    std::pair<std::vector<uint8_t>, mab::I_CommunicationInterface::Error_t>
    handleTransfer(const std::vector<uint8_t>& data, const u32 /*timeoutMs*/, const size_t expectedReceivedDataSize);

    /// @brief Handle a transfer request with no expected response
    mab::I_CommunicationInterface::Error_t handleTransferNoResponse(const std::vector<uint8_t>& data,
                                                                    const u32 /*timeoutMs*/);

    /// @brief Clear all existing request-response mappings
    void clearMappings();

    /// @brief Wait until at least N transfers have been observed or timeout elapsed
    bool waitForTransfers(size_t N, std::chrono::milliseconds timeout);

    /// @brief Get the number of observed transfers
    size_t observedTransfers() const;

    /// @brief Enable or disable automatic padding of responses to expected size
    void setAutoPadResponses(bool enable) { m_autoPad = enable; }

   private:
    struct Mapping {
        Predicate pred;
        std::vector<uint8_t> resp;
        mab::I_CommunicationInterface::Error_t err;
    };

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    std::vector<Mapping> m_mappings;
    std::atomic<size_t> m_counter{0};
    bool m_autoPad = true;
};
