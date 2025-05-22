bool PubSubClient::publish_P(const char* topic, const uint8_t* payload, unsigned int plength, bool retained) {
    // ...existing code...
    int expectedLength = 5 + 2 + topicLength + plength;
    // 修复类型不匹配警告 / Fix type mismatch warning
    return (static_cast<unsigned int>(rc) == static_cast<unsigned int>(expectedLength));
}