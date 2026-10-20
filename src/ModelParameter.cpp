#include <sstream>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ModelParameter.h"
#include "ICubeModelParameter.h"
#include "SCubeModelParameter.h"

std::shared_ptr<ModelParameter> ModelParameter::create(meere::sensor::sptr_camera camera) {
    if (camera->source()->name() == "S100D"
        || camera->source()->name() == "S110D"
        || camera->source()->name() == "S111D"
        || camera->source()->name() == "E100") {
        return std::make_shared<SCubeModelParameter>(camera);
    }
    else if (camera->source()->name() == "I200D") {
        return std::make_shared<ICubeModelParameter>(camera);
    }

    return nullptr;
}

void ModelParameter::build(ModelParameter::Descriptor descriptors[], size_t len) {
    for (size_t i = 0; i < len; i++) {
        mDescriptors.push_back(descriptors[i]);
    }
}

bool ModelParameter::readCameraValue(const ModelParameter::Descriptor& desc, int& out_value) const {
    int value = 0;
    auto _rt = mCamera->getProperty(desc.name);
    auto _prop = std::get<1>(_rt);
    if (std::get<0>(_rt) != meere::sensor::result::success || _prop == nullptr) {
        return false;
    }

    switch (desc.data_type) {
    case meere::sensor::DataType::Boolean:
        value = _prop->asBoolean() ? 1 : 0;
        break;
    case meere::sensor::DataType::U16:
        value = _prop->asInt16u();
        break;
    case meere::sensor::DataType::U8:
        value = _prop->asInt8u();
        break;
    case meere::sensor::DataType::S16:
        value = _prop->asInt16s();
        break;
    case meere::sensor::DataType::S8:
        value = _prop->asInt8s();
        break;
    default:
        return false;
    }

    // keep a value the ROS parameter range would reject out of the declaration
    if (desc.param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER
        && (value < desc.from_value || value > desc.to_value)) {
        return false;
    }
    out_value = value;
    return true;
}

meere::sensor::sptr_property ModelParameter::makeProperty(const ModelParameter::Descriptor& desc, const rclcpp::Parameter& parameter) const {
    switch (desc.data_type) {
    case meere::sensor::DataType::Boolean:
        return meere::sensor::make_property_bool(desc.name, parameter.as_bool());
    case meere::sensor::DataType::U16:
        return meere::sensor::make_property_16u(desc.name, parameter.as_int());
    case meere::sensor::DataType::U8:
        return meere::sensor::make_property_8u(desc.name, parameter.as_int());
    case meere::sensor::DataType::S16:
        return meere::sensor::make_property_16s(desc.name, parameter.as_int());
    case meere::sensor::DataType::S8:
        return meere::sensor::make_property_8s(desc.name, parameter.as_int());
    }
    return nullptr;
}

void ModelParameter::addTo(rclcpp::Node* node) {
    // declare parameters, defaulting to the value currently stored in the camera.
    // a value given at launch (parameter override) wins and is written to the camera.
    for (const auto & _desc : mDescriptors) {
        std::string _name = _desc.name;

        int _camera_value = _desc.default_value;
        bool _has_camera_value = readCameraValue(_desc, _camera_value);

        rcl_interfaces::msg::ParameterDescriptor _descriptor;
        _descriptor.name = _desc.name;
        _descriptor.type = _desc.param_type;
        _descriptor.description = _desc.description;
        _descriptor.dynamic_typing = true;

        int _declared_value = _camera_value;
        if (_desc.param_type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
            rcl_interfaces::msg::IntegerRange _range;
            _range.from_value = _desc.from_value;
            _range.to_value = _desc.to_value;
            _range.step = _desc.step;
            _descriptor.integer_range.push_back(_range);

            _declared_value = (int)node->declare_parameter(_name, _camera_value, _descriptor);
        }
        else if (_desc.param_type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
            _declared_value = node->declare_parameter(_name, _camera_value != 0, _descriptor) ? 1 : 0;
        }
        else {
            continue;
        }

        // write only what differs from the camera; with an unreadable camera value, only explicit launch values
        bool _overridden = node->get_node_parameters_interface()->get_parameter_overrides().count(_name) > 0;
        if (_has_camera_value ? (_declared_value == _camera_value) : !_overridden) {
            continue;
        }

        auto _prop = makeProperty(_desc, node->get_parameter(_name));
        if (_prop != nullptr && meere::sensor::result::success == mCamera->setProperty(_prop)) {
            RCLCPP_INFO(node->get_logger(), "camera property %s = %d", _name.c_str(), _declared_value);
        }
        else {
            RCLCPP_WARN(node->get_logger(), "setProperty(%s = %d) failed, camera keeps %d", _name.c_str(), _declared_value, _camera_value);
        }
    }

    mCallbackHandle = node->add_on_set_parameters_callback(std::bind(&ModelParameter::parametersCallback, this, std::placeholders::_1));    
}

rcl_interfaces::msg::SetParametersResult ModelParameter::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult _result;
    _result.successful = true;
    _result.reason = "success";
    for (const auto &_parameter : parameters)
    {
        auto it = std::find_if(mDescriptors.begin(), mDescriptors.end(), 
                                        [_parameter](const ModelParameter::Descriptor & desc) -> bool { return desc.name == _parameter.get_name(); });

        // not a camera property (e.g. node parameters): leave it to the node
        if (it == mDescriptors.end()) {
            continue;
        }

        meere::sensor::sptr_property _prop = makeProperty(*it, _parameter);
        if (_prop != nullptr) {
            if (meere::sensor::result::success != mCamera->setProperty(_prop)) {
                _result.successful = false;
                _result.reason = "setProperty(" + _prop->key() + ") is failed.";
                if ((*it).name == "framerate") {
                    _result.reason += " framerate can only be changed while the camera is stopped.";
                }
                break;
            }
        }
    }
    return _result;
}

void ModelParameter::removeFrom(rclcpp::Node* node) {
    for (const auto & _desc : mDescriptors) {
        std::string _name = _desc.name;
        node->undeclare_parameter(_name);
    }

    node->remove_on_set_parameters_callback(mCallbackHandle.get());
}
