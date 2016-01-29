#ifndef LISTCHILDS_REQUEST_SPLTYPES_H
#define LISTCHILDS_REQUEST_SPLTYPES_H

#include "ccpp_ListChilds_Request_.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __ListChilds_Request__ros2_components_msg__load (c_base base);

extern c_metaObject __ListChilds_Request__ros2_components_msg_srv__load (c_base base);

extern c_metaObject __ListChilds_Request__ros2_components_msg_srv_dds___load (c_base base);

extern c_metaObject __ros2_components_msg_srv_dds__ListChilds_Request___load (c_base base);
extern const char * __ros2_components_msg_srv_dds__ListChilds_Request___keys (void);
extern const char * __ros2_components_msg_srv_dds__ListChilds_Request___name (void);
struct _ros2_components_msg_srv_dds__ListChilds_Request_ ;
extern  c_bool __ros2_components_msg_srv_dds__ListChilds_Request___copyIn(c_base base, struct ros2_components_msg::srv::dds_::ListChilds_Request_ *from, struct _ros2_components_msg_srv_dds__ListChilds_Request_ *to);
extern  void __ros2_components_msg_srv_dds__ListChilds_Request___copyOut(void *_from, void *_to);
struct _ros2_components_msg_srv_dds__ListChilds_Request_ {
    c_bool dummy;
};

#endif
