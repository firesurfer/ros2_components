#ifndef LISTCHILDS_RESPONSE_SPLTYPES_H
#define LISTCHILDS_RESPONSE_SPLTYPES_H

#include "ccpp_ListChilds_Response_.h"

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>

extern c_metaObject __ListChilds_Response__ros2_components_msg__load (c_base base);

extern c_metaObject __ListChilds_Response__ros2_components_msg_srv__load (c_base base);

extern c_metaObject __ListChilds_Response__ros2_components_msg_srv_dds___load (c_base base);

extern c_metaObject __ros2_components_msg_srv_dds__ListChilds_Response___load (c_base base);
extern const char * __ros2_components_msg_srv_dds__ListChilds_Response___keys (void);
extern const char * __ros2_components_msg_srv_dds__ListChilds_Response___name (void);
struct _ros2_components_msg_srv_dds__ListChilds_Response_ ;
extern  c_bool __ros2_components_msg_srv_dds__ListChilds_Response___copyIn(c_base base, struct ros2_components_msg::srv::dds_::ListChilds_Response_ *from, struct _ros2_components_msg_srv_dds__ListChilds_Response_ *to);
extern  void __ros2_components_msg_srv_dds__ListChilds_Response___copyOut(void *_from, void *_to);
struct _ros2_components_msg_srv_dds__ListChilds_Response_ {
    c_sequence childids_;
    c_sequence childtypes_;
};

#endif
