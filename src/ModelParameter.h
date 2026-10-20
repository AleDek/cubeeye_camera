#ifndef MODEL_PARAMETER_H_
#define MODEL_PARAMETER_H_

class ModelParameter
{
public:
    struct Descriptor {
        std::string name;
        std::string description;
        uint8_t param_type;
        int data_type;
        int from_value;
        int to_value;
        int step;
        int default_value;
    };

    ModelParameter(meere::sensor::sptr_camera camera) : mCamera(camera) {};

    virtual void addTo(rclcpp::Node* node);
    virtual rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    virtual void removeFrom(rclcpp::Node* node);

    static std::shared_ptr<ModelParameter> create(meere::sensor::sptr_camera camera);

protected:
    void build(ModelParameter::Descriptor descriptors[], size_t len);
    bool readCameraValue(const ModelParameter::Descriptor& desc, int& value) const;
    meere::sensor::sptr_property makeProperty(const ModelParameter::Descriptor& desc, const rclcpp::Parameter& parameter) const;

protected:
    meere::sensor::sptr_camera mCamera;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr mCallbackHandle;

    std::vector<ModelParameter::Descriptor> mDescriptors;
};

#endif // MODEL_PARAMETER_H_
