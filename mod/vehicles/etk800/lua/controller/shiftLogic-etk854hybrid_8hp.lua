-- ETK 854 ActiveHybrid - loader shim for the etk854hybrid_8hp shift logic.
-- vehicleController.lua:1117-1124 only looks for
-- lua/vehicle/controller/vehicleController/shiftLogic/<name>.lua in the GLOBAL
-- tree (FS:fileExists); a vehicle-local module fails that check and is then
-- loaded as require("controller/shiftLogic-<name>"), which resolves here first
-- because main.lua:212 prepends <vehicle>/lua/?.lua to package.path.
-- Without this file vehicleController fails to init and the car falls back to
-- a dummy main controller (controller.lua:532-541) - i.e. undriveable.
return require("controller/vehicleController/shiftLogic/etk854hybrid_8hp")
