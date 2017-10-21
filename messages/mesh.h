namespace dmp_msgs
{
class Mesh
{
public:
  Mesh() = delete;

  explicit Mesh(std::string name)
      : name_(name)
  {
  }

  template<typename Archive>
  void serialize(Archive& ar)
  {
    ar & name_ & texture_name_ & vertex_buffer_ & normal_buffer_ & texture_buffer_ & color_buffer_ & face_buffer_;
  }

private:
  std::string name_;
  std::string texture_name_;
  std::vector<float> vertex_buffer_;
  std::vector<float> normal_buffer_;
  std::vector<float> texture_buffer_;
  std::vector<float> color_buffer_;
  std::vector<int> face_buffer_;
};
}
