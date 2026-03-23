//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_FILEREADER_H
#define CROW_SERVER_FILEREADER_H
#include "../bt_headers.h"
class FileReader: public SyncActionNode
{
public:
    FileReader(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<std::string>("json_file"),
                OutputPort<std::string>("json_str")};
    }

    NodeStatus tick() override
    {
        std::string filePath_;
        if (!getInput("json_file", filePath_))
        {
            throw RuntimeError("error reading port [json_file]: ");
        }
        std::ifstream file(filePath_);
        json data = json::parse(file);

        setOutput("json_str", data.dump());
        return NodeStatus::SUCCESS;
    }
};
#endif //CROW_SERVER_FILEREADER_H
