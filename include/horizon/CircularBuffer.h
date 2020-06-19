#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <cstdlib>
#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>

template<typename T>
class CircularBuffer {
 public:
  bool AbandonLast = false;
  explicit CircularBuffer(const size_t &capacity = 200)
      : capacity_(capacity),
        size_(0),
        start_idx_(0) {
    buffer_ = new T[capacity];
  };

  CircularBuffer(CircularBuffer<T> & r){
    capacity_ = r.capacity();
    size_ = r.size();
    start_idx_ = r.start_idx();
    buffer_ = new T[r.capacity()];
    for(size_t i=0; i<r.size(); i++){
      buffer_[(start_idx_ + i) % capacity_] = r[i];
    }
  }

  ~CircularBuffer() {
    delete[] buffer_;
    buffer_ = NULL;
  }

  void CopyFrom( CircularBuffer<T> & r ){
    std::unique_lock<std::mutex> ul(m_mut);
    delete[] buffer_;
    buffer_ = NULL;
    capacity_ = r.capacity();
    size_ = r.size();
    start_idx_ = r.start_idx();
    buffer_ = new T[capacity_];
    for(size_t i=0; i<size_; i++){
      buffer_[(start_idx_ + i) % capacity_] = r[i];
    }
  }

  void Reset(size_t capacity = 1) {
    std::unique_lock<std::mutex> ul(m_mut);
    delete[] buffer_;
    buffer_ = NULL;

    capacity_ = capacity;
    size_ = 0;
    start_idx_ = 0;
    buffer_ = new T[capacity];
  }

  void Clear() {
    std::unique_lock<std::mutex> ul(m_mut);
    delete[] buffer_;
    buffer_ = NULL;

    size_ = 0;
    start_idx_ = 0;
    buffer_ = new T[capacity_];
  }

  void Reset(size_t begin, size_t end) {
    std::unique_lock<std::mutex> ul(m_mut);
    size_t newSize = end - begin +1;
    T *buffer_new_ = new T[newSize];

    for(size_t i=begin; i<=end; ++i){
      buffer_new_[i-begin] = buffer_[(start_idx_ + i) % capacity_];
    }

    delete[] buffer_;
    buffer_ = NULL;

    capacity_ = newSize;
    size_ = newSize;
    start_idx_ = 0;
    buffer_ = buffer_new_;
  }

  const size_t &size() {
    std::unique_lock<std::mutex> ul(m_mut);
    return size_;
  }

  const size_t &capacity() {
    std::unique_lock<std::mutex> ul(m_mut);
    return capacity_;
  }

  const size_t &start_idx() {
    std::unique_lock<std::mutex> ul(m_mut);
    return start_idx_;
  }

  void EnsureCapacity(const int &req_apacity) {
    if (req_apacity > 0 && capacity_ < req_apacity) {
      // create new buffer and copy (valid) entries
      T *new_buffer = new T[req_apacity];
      for (size_t i = 0; i < size_; i++) {
        new_buffer[i] = (*this)[i];
      }

      // switch buffer pointers and delete old buffer
      T *old_buffer = buffer_;
      buffer_ = new_buffer;
      start_idx_ = 0;

      delete[] old_buffer;
    }
  }

  bool empty() {
    std::unique_lock<std::mutex> ul(m_mut);
    return size_ == 0;
  }

  T &operator[](const size_t &i) {
    return buffer_[(start_idx_ + i) % capacity_];
  }

  const T &operator[](const size_t &i) const {
    return buffer_[(start_idx_ + i) % capacity_];
  }

  const CircularBuffer<T> & operator=( CircularBuffer<T> & r )  {
    this->Reset(r.capacity());
    this->capacity_ = r.capacity();
    this->size_ = r.size();
    this->start_idx_ = r.start_idx();
    for(size_t i=0; i<r.size(); i++){
      (*this)[i] = r[i];
    }
    return *this;
  }

  const T &first() const {
    std::unique_lock<std::mutex> ul(m_mut);
    return buffer_[start_idx_];
  }

  T &first() {
    std::unique_lock<std::mutex> ul(m_mut);
    return buffer_[start_idx_];
  }

  const T &last() const {
    std::unique_lock<std::mutex> ul(m_mut);
    size_t idx = size_ == 0 ? 0 : (start_idx_ + size_ - 1) % capacity_;
    return buffer_[idx];
  }

  T &last() {
    std::unique_lock<std::mutex> ul(m_mut);
    size_t idx = size_ == 0 ? 0 : (start_idx_ + size_ - 1) % capacity_;
    return buffer_[idx];
  }

  void push(const T &element) {
    std::unique_lock<std::mutex> ul(m_mut);
    if (size_ < capacity_) {
      buffer_[size_] = element;
      ++size_;
    } else {
      buffer_[start_idx_] = element;
      start_idx_ = (start_idx_ + 1) % capacity_;
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  size_t capacity_;   //< buffer capacity
  size_t size_;       //< current buffer size
  size_t start_idx_;   //< current start index
  T *buffer_;         //< internal element buffer
  mutable std::mutex m_mut;
};

class LocalMaper{

    typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudT;

public:
    explicit LocalMaper(const size_t& width):
    width_(width), size_(0), start_idx_(0),
    SharpLocalMap(new PointCloudT()),
    FlatLocalMap(new PointCloudT()){
      vSharpIndex_.resize(width);
      vFlatIndex_.resize(width);
      SharpLocalMap->reserve(20000);
      FlatLocalMap->reserve(20000);
    }
    ~LocalMaper()= default;

    void push(const PointCloudT::Ptr& sharp, const PointCloudT::Ptr& flat){}

    const PointCloudT::Ptr& getSharpLocalMap(){return SharpLocalMap;}

    const PointCloudT::Ptr& getFlatLocalMap(){return FlatLocalMap;}

    const size_t &size() {
      std::unique_lock<std::mutex> ul(m_mut);
      return size_;
    }

private:
    PointCloudT::Ptr SharpLocalMap;
    PointCloudT::Ptr FlatLocalMap;
    const size_t width_;
    size_t size_;       //< current buffer size
    size_t start_idx_;   //< current start index
    mutable std::mutex m_mut;
    std::vector<std::vector<int>> vSharpIndex_;
    std::vector<std::vector<int>> vFlatIndex_;
    std::vector<int> qSharpHomeless;
    std::vector<int> qFlatHomeless;
};


#endif // LIO_CIRCULARBUFFER_H_
