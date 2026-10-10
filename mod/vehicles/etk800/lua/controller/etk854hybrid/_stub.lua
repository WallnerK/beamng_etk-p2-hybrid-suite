-- _stub.lua - Safe no-op fallback for failed submodules
-- If safeRequire encounters a syntax error in a real submodule, it falls back
-- to this stub so the vehicle still drives.

local M = {}

function M.init(ctx)
  if ctx and ctx.dbg then ctx.dbg(1, "stub", "stub init") end
end

function M.updateGFX(ctx, dt) end
function M.updateFixedStep(ctx, dt) end
function M.reset(ctx) end

return M
