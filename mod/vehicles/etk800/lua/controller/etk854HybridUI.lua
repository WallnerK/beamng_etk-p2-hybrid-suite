-- etk854HybridUI.lua
-- Auto-loads the ETK 854 P2 Hybrid UI layout (shipped at
-- settings/ui_apps/originalLayouts/) once when the vehicle spawns.
--
-- Mechanism: obj:queueGameEngineLua() triggers 'appContainer:loadLayoutByType'
-- (0.39 removed 'loadLayoutByFilename'); appLayoutsStore.js resolves the type
-- string against the layout's "type" field / filename stem.
-- Only the in-memory current layout changes, nothing is written to disk. On a
-- switch to another vehicle BeamNG keeps the shown layout (no revert).

local M = {}
M.type = "auxiliary"

-- layout file: /settings/ui_apps/originalLayouts/etk854_hybrid.uilayout.json (loaded by type below)
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
  loadLayout()
end

function M.initSecondStage(jbeamData)
  -- second chance if init was too early; `loaded` prevents a double load
  -- (a vehicle reset re-runs init as well)
  loadLayout()
end

return M
