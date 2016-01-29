#include "Sample_ListChilds_Response_SplDcps.h"
#include "ccpp_Sample_ListChilds_Response_.h"
#include "dds_type_aliases.h"

const char *
__ros2_components_msg_srv_dds__Sample_ListChilds_Response___name(void)
{
    return (const char*)"ros2_components_msg::srv::dds_::Sample_ListChilds_Response_";
}

const char *
__ros2_components_msg_srv_dds__Sample_ListChilds_Response___keys(void)
{
    return (const char*)"";
}

#include <v_kernel.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

c_bool
__ros2_components_msg_srv_dds__Sample_ListChilds_Response___copyIn(
    c_base base,
    struct ::ros2_components_msg::srv::dds_::Sample_ListChilds_Response_ *from,
    struct _ros2_components_msg_srv_dds__Sample_ListChilds_Response_ *to)
{
    c_bool result = OS_C_TRUE;
    (void) base;

    to->client_guid_0_ = (c_ulonglong)from->client_guid_0_;
    to->client_guid_1_ = (c_ulonglong)from->client_guid_1_;
    to->sequence_number_ = (c_longlong)from->sequence_number_;
    if(result){
        extern c_bool __ros2_components_msg_srv_dds__ListChilds_Response___copyIn(c_base, ::ros2_components_msg::srv::dds_::ListChilds_Response_ *, _ros2_components_msg_srv_dds__ListChilds_Response_ *);
        result = __ros2_components_msg_srv_dds__ListChilds_Response___copyIn(base, &from->response_, &to->response_);
    }
    return result;
}

void
__ros2_components_msg_srv_dds__Sample_ListChilds_Response___copyOut(
    void *_from,
    void *_to)
{
    struct _ros2_components_msg_srv_dds__Sample_ListChilds_Response_ *from = (struct _ros2_components_msg_srv_dds__Sample_ListChilds_Response_ *)_from;
    struct ::ros2_components_msg::srv::dds_::Sample_ListChilds_Response_ *to = (struct ::ros2_components_msg::srv::dds_::Sample_ListChilds_Response_ *)_to;
    to->client_guid_0_ = (::DDS::ULongLong)from->client_guid_0_;
    to->client_guid_1_ = (::DDS::ULongLong)from->client_guid_1_;
    to->sequence_number_ = (::DDS::LongLong)from->sequence_number_;
    {
        extern void __ros2_components_msg_srv_dds__ListChilds_Response___copyOut(void *, void *);
        __ros2_components_msg_srv_dds__ListChilds_Response___copyOut((void *)&from->response_, (void *)&to->response_);
    }
}

