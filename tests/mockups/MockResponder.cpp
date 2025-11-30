// Copyright (c) 2025, Błażej Szargut.
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#include "MockResponder.hpp"

#include <algorithm>
#include <cstring>

#include "md_types.hpp"

MockResponder::MockResponder() = default;
MockResponder::~MockResponder() = default;

void MockResponder::addFixedResponse(const std::vector<uint8_t>& requestPrefix,
                                     const std::vector<uint8_t>& response,
                                     mab::I_CommunicationInterface::Error_t err) {
    std::lock_guard<std::mutex> lk(m_mutex);
    Mapping m;
    m.pred = [requestPrefix](const std::vector<uint8_t>& d) {
        if (d.size() < requestPrefix.size())
            return false;
        return std::equal(requestPrefix.begin(), requestPrefix.end(), d.begin());
    };
    m.resp = response;
    m.err = err;
    m_mappings.push_back(std::move(m));
}

void MockResponder::addPredicateResponse(Predicate pred,
                                         const std::vector<uint8_t>& response,
                                         mab::I_CommunicationInterface::Error_t err) {
    std::lock_guard<std::mutex> lk(m_mutex);
    Mapping m;
    m.pred = std::move(pred);
    m.resp = response;
    m.err = err;
    m_mappings.push_back(std::move(m));
}

std::pair<std::vector<uint8_t>, mab::I_CommunicationInterface::Error_t>
MockResponder::handleTransfer(const std::vector<uint8_t>& data, const u32 /*timeoutMs*/, const size_t expectedReceivedDataSize) {
    std::vector<uint8_t> resp;
    mab::I_CommunicationInterface::Error_t err = mab::I_CommunicationInterface::UNKNOWN_ERROR;

    // First try explicit mappings
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        for (const auto& m : m_mappings) {
            try {
                if (m.pred(data)) {
                    resp = m.resp;
                    err = m.err;
                    break;
                }
            } catch (...) {
            }
        }
    }

    // If no mapping matched, try to detect inner MD commands inside Candle frame payload
    if (resp.empty()) {
        auto it = std::find(data.begin(), data.end(), static_cast<uint8_t>(mab::MdFrameId_E::READ_REGISTER));
        if (it != data.end()) {
            size_t idx = std::distance(data.begin(), it);
            // ensure there are at least two bytes after header byte for address
            if (idx + 3 < data.size()) {
                uint8_t addr_lsb = data[idx + 2];
                uint8_t addr_msb = data[idx + 3];
                // craft response: Candle header (0x04,0x01), then READ_REGISTER(0x41), 0x00, addr_lsb, addr_msb, <4-byte payload>
                resp = std::vector<uint8_t>{0x04, 0x01, static_cast<uint8_t>(mab::MdFrameId_E::READ_REGISTER), 0x00, addr_lsb, addr_msb, 0x00, 0x00, 0x00, 0x00};
                err = mab::I_CommunicationInterface::OK;
            }
        }
    }

    // If still no mapping, detect WRITE_REGISTER_LEGACY (0x40)
    if (resp.empty()) {
        auto itw = std::find(data.begin(), data.end(), static_cast<uint8_t>(mab::MdFrameId_E::WRITE_REGISTER_LEGACY));
        if (itw != data.end()) {
            resp = std::vector<uint8_t>{0x04, 0x01, static_cast<uint8_t>(mab::MdFrameId_E::RESPONSE_LEGACY), 0x00};
            err = mab::I_CommunicationInterface::OK;
        }
    }

    // Fallback: detect Candle datarate config (0x02 = CANDLE_CONFIG_DATARATE)
    if (resp.empty() && !data.empty() && data[0] == 0x02) {
        // Candle version reply format expected by legacyCheckConnection:
        // {tag, revision, minor, major} where major >= 2 and minor >= 4 for MD compatibility
        resp = {0x04, 0x01, 0x00, 0x00, 0x04, 0x02};
        err = mab::I_CommunicationInterface::OK;
    }

    // Final fallback generic response
    if (resp.empty()) {
        resp = {0x04, 0x01, 0xA0, 0x00};
        err = mab::I_CommunicationInterface::OK;
    }

    if (m_autoPad && expectedReceivedDataSize > resp.size()) {
        resp.resize(expectedReceivedDataSize, 0x00);
    }

    m_counter.fetch_add(1);
    m_cv.notify_all();

    return std::make_pair(resp, err);
}

mab::I_CommunicationInterface::Error_t MockResponder::handleTransferNoResponse(const std::vector<uint8_t>& /*data*/, const u32 /*timeoutMs*/) {
    m_counter.fetch_add(1);
    m_cv.notify_all();
    return mab::I_CommunicationInterface::OK;
}

void MockResponder::clearMappings() {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_mappings.clear();
}

bool MockResponder::waitForTransfers(size_t N, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(m_mutex);
    return m_cv.wait_for(lk, timeout, [this, N]() { return m_counter.load() >= N; });
}

size_t MockResponder::observedTransfers() const {
    return m_counter.load();
}
