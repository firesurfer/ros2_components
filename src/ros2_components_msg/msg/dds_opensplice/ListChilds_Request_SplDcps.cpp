#include "ListChilds_Request_SplDcps.h"
#include "ccpp_ListChilds_Request_.h"
#include "dds_type_aliases.h"

const char *
__ros2_components_msg_srv_dds__ListChilds_Request___name(void)
{
    return (const char*)"ros2_components_msg::srv::dds_::ListChilds_Request_";
}

const char *
__ros2_components_msg_srv_dds__ListChilds_Request___keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__ros2_components_msg_srv_dds__ListChilds_Request___copyIn(
    c_base base,
    struct ::ros2_components_msg::srv::dds_::ListChilds_Request_ *from,
    struct _ros2_components_msg_srv_dds__ListChilds_Request_ *to)
{
    c_bool result = OS_C_TRUE;
    (void) base;

    to->dummy = (c_bool)from->dummy;
    return result;
}

void
__ros2_components_msg_srv_dds__ListChilds_Request___copyOut(
    void *_from,
    void *_to)
{
    struct _ros2_components_msg_srv_dds__ListChilds_Request_ *from = (struct _ros2_components_msg_srv_dds__ListChilds_Request_ *)_from;
    struct ::ros2_components_msg::srv::dds_::ListChilds_Request_ *to = (struct ::ros2_components_msg::srv::dds_::ListChilds_Request_ *)_to;
    to->dummy = (::DDS::Boolean)(from->dummy != 0);
}

