-- etk854HybridUI.lua
-- Auto-loads the ETK 854 P2 Hybrid UI layout when the vehicle spawns.
-- The layout file is shipped with the mod at settings/ui_apps/originalLayouts/.
--
-- Mechanism: vehicle controllers can call into the GameEngine Lua via
-- obj:queueGameEngineLua(). We trigger 'appContainer:loadLayoutByType' -
-- 0.39 removed 'loadLayoutByFilename'; appLayoutsStore.js resolves the type
-- string against our layout's "type" field / filename stem.
--
-- This affects only the IN-MEMORY current layout; it does not write to disk.
-- When the player switches to a different vehicle, BeamNG keeps whatever layout
-- is currently shown (no automatic revert), which is acceptable for our case.

local M = {}
M.type = "auxiliary"

local LAYOUT_PATH = "/settings/ui_apps/originalLayouts/etk854_hybrid.uilayout.json" -- informational; loaded by type below
local loaded = false

local function loadLayout()
  if loaded then return end
  loaded = true
  obj:queueGameEngineLua([[
    guihooks.trigger('appContainer:loadLayoutByType', 'etk854_hybrid')
    log('I', 'etk854HybridUI', 'Auto-loaded hybrid UI layout')
  ]])
end

function M.init(jbeamData)
  -- Called when vehicle is fully spawned. Trigger the layout load.
  loadLayout()
end

function M.initSecondStage(jbeamData)
  -- Belt-and-suspenders: if init was too early, try again at initStage2.
  -- The `loaded` flag prevents double application.
  loadLayout()
end

return M
