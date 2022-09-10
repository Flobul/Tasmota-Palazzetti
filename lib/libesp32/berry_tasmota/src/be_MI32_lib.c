/********************************************************************
 * Tasmota lib
 * 
 * To use: `import MI32`
 *******************************************************************/
#include "be_constobj.h"
#include "be_mapping.h"

#ifdef USE_MI_ESP32
extern int be_MI32_devices(bvm *vm);
extern int be_MI32_set_bat(bvm *vm);
extern int be_MI32_get_name(bvm *vm);
extern int be_MI32_get_MAC(bvm *vm);
extern int be_MI32_set_hum(bvm *vm);
extern int be_MI32_set_temp(bvm *vm);

#include "be_fixed_MI32.h"

/* @const_object_info_begin
module MI32 (scope: global) {
  devices,    func(be_MI32_devices)
  get_name,   func(be_MI32_get_name)
  get_MAC,    func(be_MI32_get_MAC)
  set_bat,    func(be_MI32_set_bat)
  set_hum,    func(be_MI32_set_hum)
  set_temp,   func(be_MI32_set_temp)
}
@const_object_info_end */

/********************************************************************
 * Tasmota lib
 * 
 * To use: `import BLE`
 *******************************************************************/

extern void be_BLE_reg_conn_cb(void* function, uint8_t *buffer);
BE_FUNC_CTYPE_DECLARE(be_BLE_reg_conn_cb, "", "cc");

extern void be_BLE_reg_adv_cb(void* function, uint8_t *buffer);
BE_FUNC_CTYPE_DECLARE(be_BLE_reg_adv_cb, "", "c[c]");

extern void be_BLE_set_MAC(struct bvm *vm, uint8_t *buf, size_t size, uint8_t type);
BE_FUNC_CTYPE_DECLARE(be_BLE_set_MAC, "", "@(bytes)~[i]");

extern void be_BLE_set_characteristic(struct bvm *vm, const char *Chr);
BE_FUNC_CTYPE_DECLARE(be_BLE_set_characteristic, "", "@s");

extern void be_BLE_run(struct bvm *vm, uint8_t operation, bbool response);
BE_FUNC_CTYPE_DECLARE(be_BLE_run, "", "@i[b]");

extern void be_BLE_set_service(struct bvm *vm, const char *Svc, bbool discoverAttributes);
BE_FUNC_CTYPE_DECLARE(be_BLE_set_service, "", "@s[b]");

extern void be_BLE_adv_watch(struct bvm *vm, uint8_t *buf, size_t size, uint8_t type);
BE_FUNC_CTYPE_DECLARE(be_BLE_adv_watch, "", "@(bytes)~[i]");

extern void be_BLE_adv_block(struct bvm *vm, uint8_t *buf, size_t size, uint8_t type);
BE_FUNC_CTYPE_DECLARE(be_BLE_adv_block, "", "@(bytes)~[i]");


#include "be_fixed_BLE.h"

/* @const_object_info_begin
module BLE (scope: global) {
  conn_cb,    ctype_func(be_BLE_reg_conn_cb)
  set_svc,    ctype_func(be_BLE_set_service)
  run,        ctype_func(be_BLE_run)
  set_chr,    ctype_func(be_BLE_set_characteristic)
  adv_cb,     ctype_func(be_BLE_reg_adv_cb)
  set_MAC,    ctype_func(be_BLE_set_MAC)
  adv_watch,  ctype_func(be_BLE_adv_watch)
  adv_block,  ctype_func(be_BLE_adv_block)
}
@const_object_info_end */

#endif //USE_MI_ESP32