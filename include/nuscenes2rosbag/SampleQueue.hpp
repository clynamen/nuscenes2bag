#pragma once

#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <optional>
#include <sensor_msgs/Image.h>
#include <string>
#include <functional>
#include <tuple>

constexpr size_t DEFAULT_QUEUE_SIZE = 20;

template <typename T> using queue_impl = boost::lockfree::spsc_queue<T>;

template <typename T> class SampleQueueProducer {

public:
  void push(T &&element) { queue.push(std::move(element)); }
  bool canPush() { return queue.write_available() > 0; }
  void close() { closed = true; }

  SampleQueueProducer(queue_impl<T> &queue, bool &closed)
      : queue(queue), closed(closed) {}

private:
  queue_impl<T> &queue;
  bool& closed;
};

template <typename T> class SampleQueueConsumer {

public:
  SampleQueueConsumer()
      : queue(std::make_unique<queue_impl<T>>(DEFAULT_QUEUE_SIZE)),
        closed(std::make_unique<bool>(false))
      {};

  SampleQueueConsumer(const SampleQueueConsumer &r) = delete;
  SampleQueueConsumer(SampleQueueConsumer &&r)
      : queue(std::move(r.queue)), closed(std::move(r.closed)){

                                   };

  std::optional<T> get() {
    if (size()) {
      T value;
       queue->pop(value);
      return std::optional<T>(value);
    } else {
      return std::nullopt;
    }
  }

  bool isClosed() const { 
      return *closed; 
  };

  size_t size() const { 
      // std::cout  << "Size " << (int) queue->read_available() << std::endl;
      return queue->read_available(); 
  }

  std::unique_ptr<queue_impl<T>> queue;
  std::unique_ptr<bool> closed;
};

template <typename T> class SampleQueueFactory {
public:
  static auto makeQueue() {
    SampleQueueConsumer<T> consumer;
    SampleQueueProducer<T> producer(*consumer.queue, *consumer.closed);
    return std::make_pair(std::move(producer), std::move(consumer));
  }
};

struct TopicInfo {
  std::string topicName;

  TopicInfo(const std::string &topicName);
};

TopicInfo::TopicInfo(const std::string &topicName) : topicName(topicName) {}

class SampleMsgProcessor {
public:
  virtual void
  process(const TopicInfo &topicInfo,
          SampleQueueConsumer<sensor_msgs::Image> &queueConsumer) = 0;

  virtual ~SampleMsgProcessor() = default;
};

class TypeErasedQueue {

public:
  template <typename T>
  TypeErasedQueue(T &obj)
      : object(std::make_unique<TypeErasedQueueModel<T>>(std::move(obj))) {}

  void process(const TopicInfo &topicInfo,
               SampleMsgProcessor &processor) {
    return object->process(topicInfo, processor);
  }

  bool isClosed() const { return object->isClosed(); }

  size_t size() const { return object->size(); }

  struct TypeErasedQueueConcept {
    virtual ~TypeErasedQueueConcept() {}
    virtual void process(const TopicInfo &topicInfo,
                         SampleMsgProcessor &processor) = 0;
    virtual bool isClosed() const = 0;
    virtual size_t size() const = 0;
  };

  template <typename T> struct TypeErasedQueueModel : TypeErasedQueueConcept {
    TypeErasedQueueModel(T &&t) : object(std::move(t)) {}

    void process(const TopicInfo &topicInfo,
                 SampleMsgProcessor &processor) override {
      processor.process(topicInfo, object);
    }

    bool isClosed() const override { return object.isClosed(); }

    size_t size() const override { return object.size(); }

  private:
    T object;
  };

  std::unique_ptr<TypeErasedQueueConcept> object;
};
