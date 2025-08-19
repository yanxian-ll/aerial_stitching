#ifndef RABBITMQ_H
#define RABBITMQ_H
#include <iostream>
#include <string>
#include <cstdlib>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>
#include <boost/asio.hpp>
#include <amqpcpp.h>
#include <amqpcpp/libboostasio.h>

#include "debug_utils.h"

using json = nlohmann::json;

// 获取环境变量或默认值
static std::string get_env(const std::string &key, const std::string &defv)
{
    if (const char *v = std::getenv(key.c_str()))
        return std::string(v);
    return defv;
}

class RabbitMQManager
{
public:
    RabbitMQManager()
    {
        host = get_env("RABBIT_HOST", "rabbitmq-host");
        port = std::stoi(get_env("RABBIT_PORT", "5672"));
        username = get_env("RABBIT_USERNAME", "admin");
        password = get_env("RABBIT_PASSWORD", "secret");
        exchange = get_env("RABBIT_EXCHANGE", "topic.kuaipin");
        routing_key = get_env("RABBIT_ROUTINGKEY", "kuaipin.zn");
        queue = get_env("RABBIT_QUEUE", "kuaipin");
        vhost = get_env("RABBIT_VHOST", "/");

        std::cout << std::endl;
        LOG_INFO("------- RabbitMQ INFO -------");
        LOG_INFO("host:        " + host);
        LOG_INFO("port:        " + std::to_string(port));
        LOG_INFO("username:    " + username);
        LOG_INFO("password:    " + password);
        LOG_INFO("exchange:    " + exchange);
        LOG_INFO("routing_key: " + routing_key);
        LOG_INFO("queue:       " + queue);
        LOG_INFO("vhost:       " + vhost);
        std::cout << "------------------------------\n\n";
    }

    // 声明交换机/队列并绑定
    bool init_rabbitmq_infrastructure()
    {
        try
        {
            handler = std::make_unique<AMQP::LibBoostAsioHandler>(io);
            address = std::make_unique<AMQP::Address>(host, port, AMQP::Login(username, password), vhost);

            connection = std::make_unique<AMQP::TcpConnection>(handler.get(), *address);
            channel = std::make_unique<AMQP::TcpChannel>(connection.get());

            bool ok = true;
            std::promise<void> init_done;
            auto future = init_done.get_future();

            channel->onError([&](const char *msg)
                             {
                LOG_ERROR("[×] RabbitMQ 初始化失败: " + std::string(msg));
                ok = false;
                init_done.set_value(); });

            // 声明交换机
            channel->declareExchange(exchange, AMQP::ExchangeType::topic, AMQP::durable)
                .onSuccess([&]()
                           {
                    LOG_INFO("[✓] 交换机声明成功: " + exchange);
                    // 声明队列
                    channel->declareQueue(queue, AMQP::durable)
                        .onSuccess([&](const std::string&, uint32_t, uint32_t) {
                            LOG_INFO("[✓] 队列声明成功: " + queue);
                            // 绑定
                            channel->bindQueue(exchange, queue, routing_key)
                                .onSuccess([&]() {
                                    LOG_INFO("[✓] 绑定成功: " + exchange + " -> " + queue + "(routing_key=" + routing_key + ")");
                                    init_done.set_value();
                                })
                                .onError([&](const char* msg) {
                                    LOG_ERROR("[×] 绑定失败: " + std::string(msg));
                                    ok = false;
                                    init_done.set_value();
                                });
                        })
                        .onError([&](const char* msg) {
                            LOG_ERROR("[×] 队列声明失败: " + std::string(msg));
                            ok = false;
                            init_done.set_value();
                        }); })
                .onError([&](const char *msg)
                         {
                    LOG_ERROR("[×] 交换机声明失败: " + std::string(msg));
                    ok = false;
                    init_done.set_value(); });

            // 启动事件循环线程
            running = true;
            io_thread = std::thread([this]()
                                    { io.run(); });

            // 等待初始化完成
            future.wait();

            if (ok)
                LOG_INFO("[✓] RabbitMQ 初始化完成: " + exchange + "/" + routing_key + " -> " + queue);
            return ok;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("[×] RabbitMQ 初始化异常: " + std::string(e.what()));
            return false;
        }
    }

    // 发送 JSON 消息
    bool send_model_progress(const json &payload)
    {
        if (!channel)
        {
            LOG_ERROR("[×] RabbitMQ Channel 未初始化");
            return false;
        }

        try
        {
            const std::string msg = payload.dump();
            AMQP::Envelope env(msg.data(), msg.size());
            env.setDeliveryMode(2); // 持久化
            env.setContentType("application/json");

            channel->publish(exchange, routing_key, env);
            LOG_INFO("[✓] RabbitMQ 消息已发布");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("[×] 发布消息异常: " + std::string(e.what()));
            return false;
        }
    }

    // 停止并释放资源
    void stop() {
        if (running) {
            running = false;
            try {
                if (connection) connection->close();
                io.stop();
                if (io_thread.joinable()) io_thread.join();
            }
            catch (...) {}
        }
    }

private:
    // 配置
    std::string host;
    int port{};
    std::string username, password;
    std::string exchange, routing_key, queue, vhost;

    // RabbitMQ 相关
    boost::asio::io_context io;
    std::unique_ptr<AMQP::LibBoostAsioHandler> handler;
    std::unique_ptr<AMQP::Address> address;
    std::unique_ptr<AMQP::TcpConnection> connection;
    std::unique_ptr<AMQP::TcpChannel> channel;

    // 线程控制
    std::thread io_thread;
    std::atomic<bool> running;
};

#endif // RABBITMQ_H
