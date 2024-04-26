/**
* @file     communication_manager_node.cpp
* @brief    communication_manager_node�p�b�P�[�W��main�֐�
* @author   S.Kumada
* @date     2023/12/7
* @note     �m�[�h�̏������A�}�l�[�W���̏��������s��
*/

#include "communication_manager/communication_manager.hpp"

int main(int argc, char* argv[])
{    
    ros::init(argc, argv, "communication_manager_node");
    ros::NodeHandle node;

    CommunicationManager manager(node);

    manager.manageCom();

    return (0);
}