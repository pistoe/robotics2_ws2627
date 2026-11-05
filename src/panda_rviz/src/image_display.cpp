// SPDX-License-Identifier: BSD-3-Clause
#include <message_filters/connection.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>

namespace panda_rviz {
class ImageDisplay final : public rviz_default_plugins::displays::ImageDisplay {
 public:
  ~ImageDisplay() override { unsubscribe(); }

 protected:
  void unsubscribe() override {
    if (subscription_) {
      subscription_callback_.disconnect();
      // Connection retains a raw filter pointer after disconnect in Jazzy.
      subscription_callback_ = message_filters::Connection();
      subscription_.reset();
    }
  }
  void onDisable() override {
    unsubscribe();
    rviz_default_plugins::displays::ImageDisplay::onDisable();
  }
};
}

PLUGINLIB_EXPORT_CLASS(panda_rviz::ImageDisplay, rviz_common::Display)
