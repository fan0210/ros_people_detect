#include <main.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_detect_node");

    Main * main_node = new Main();

    main_node->process();

    delete main_node;

    return 0;
}
