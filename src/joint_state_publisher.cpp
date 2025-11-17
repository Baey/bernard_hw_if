#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "candle.hpp"
#include "MD.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bernard/actuators.hpp"


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	mab::Candle* candle = mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_8M,
								mab::candleTypes::busTypes_t::USB);
	auto ids = mab::MD::discoverMDs(candle);
	std::vector<mab::MD> mds;
	for (auto &id : ids)
	{
		mab::MD md(id, candle);
        if (md.init() == mab::MD::Error_t::OK) {
			mds.push_back(md);
			RCLCPP_INFO(rclcpp::get_logger("actuator_state_publisher"), "Found drive %s, (ID: %d)", JOINT_MAP.at(id).c_str(), id);
		} else {
			RCLCPP_WARN(rclcpp::get_logger("actuator_state_publisher"), "Drive with ID %d failed to initialize", id);
		}
	}

	for (auto &md : mds)
	{
		md.enable();
	}

	auto state_pub = std::make_shared<ActuatorStatePublisher>(candle, mds);

	candle->begin();

	rclcpp::spin(state_pub);

	for (auto &md : mds)
	{
		md.disable();
	}

	mab::detachCandle(candle);
	rclcpp::shutdown();

	return 0;
}