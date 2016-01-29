#ifndef SAMPLE_LISTCHILDS_REQUEST_SPLTYPES_H
#define SAMPLE_LISTCHILDS_REQUEST_SPLTYPES_H

#include "ccpp_Sample_ListChilds_Request_.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

#include "ListChilds_Request_SplDcps.h"

extern c_metaObject __Sample_ListChilds_Request__ros2_components_msg__load (c_base base);

extern c_metaObject __Sample_ListChilds_Request__ros2_components_msg_srv__load (c_base base);

extern c_metaObject __Sample_ListChilds_Request__ros2_components_msg_srv_dds___load (c_base base);

extern c_metaObject __ros2_components_msg_srv_dds__Sample_ListChilds_Request___load (c_base base);
extern const char * __ros2_components_msg_srv_dds__Sample_ListChilds_Request___keys (void);
extern const char * __ros2_components_msg_srv_dds__Sample_ListChilds_Request___name (void);
struct _ros2_components_msg_srv_dds__Sample_ListChilds_Request_ ;
extern  c_bool __ros2_components_msg_srv_dds__Sample_ListChilds_Request___copyIn(c_base base, struct ros2_components_msg::srv::dds_::Sample_ListChilds_Request_ *from, struct _ros2_components_msg_srv_dds__Sample_ListChilds_Request_ *to);
extern  void __ros2_components_msg_srv_dds__Sample_ListChilds_Request___copyOut(void *_from, void *_to);
struct _ros2_components_msg_srv_dds__Sample_ListChilds_Request_ {
    c_ulonglong client_guid_0_;
    c_ulonglong client_guid_1_;
    c_longlong sequence_number_;
    struct _ros2_components_msg_srv_dds__ListChilds_Request_ request_;
};

#endif
