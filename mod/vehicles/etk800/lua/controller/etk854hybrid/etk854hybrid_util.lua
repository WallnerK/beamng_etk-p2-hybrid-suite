-- ETK 854 ActiveHybrid - shared stateless helpers of the vehicle controllers
--
-- Loaded with require("controller/etk854hybrid/etk854hybrid_util"). A user keeps a file-level
-- alias (local clamp = U.clamp), so call sites and upvalue counts stay as they were.
-- Only exact copies live here: a variant with different NaN / string / boolean handling is a
-- separate function below, or stays in its own file.

local M = {}

local min, max = math.min, math.max
local huge = math.huge

-- v limited to [lo, hi] by comparison (a NaN v passes through)
local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

-- min(max(x, a), b): LuaJIT's math.min / math.max NaN rules, not those of clamp
local function clampMinMax(x, a, b)
  return min(max(x, a), b)
end

-- v limited to [0, 1] (a NaN v passes through)
local function clamp01(v)
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

-- v limited to [0, 1], NaN -> 0
local function clamp01NanZero(v)
  if v ~= v then return 0 end
  if v < 0 then return 0 end
  if v > 1 then return 1 end
  return v
end

-- cubic smoothstep of t limited by clamp01NanZero (NaN -> 0)
local function smoothstep(t)
  t = clamp01NanZero(t)
  return t * t * (3 - 2 * t)
end

-- electrics flag: true or a number > 0.5
local function flag(v)
  return v == true or (type(v) == "number" and v > 0.5)
end

-- number as is (NaN / inf included), numeric string via tonumber, else d
local function num(v, d)
  if type(v) == "number" then return v end
  if type(v) == "string" then return tonumber(v) or d end
  return d
end

-- finite number (strings via tonumber), else d
local function finNum(x, d)
  if type(x) == "string" then x = tonumber(x) end
  if type(x) == "number" and x == x and x > -huge and x < huge then return x end
  return d
end

-- finite number, else 0
local function fin(x)
  if type(x) == "number" and x == x and x > -huge and x < huge then return x end
  return 0
end

-- clamped linear scale (NaN x -> y0)
local function lin(x, x0, x1, y0, y1)
  if x1 == x0 then return (x >= x1) and y1 or y0 end
  local t = (x - x0) / (x1 - x0)
  if not (t > 0) then t = 0 elseif t > 1 then t = 1 end
  return y0 + (y1 - y0) * t
end

local function sgn(x)
  if x > 0 then return 1 elseif x < 0 then return -1 end
  return 0
end

-- switch: boolean as is, number / string >= 0.5 (through finNum, so NaN / inf -> d)
local function sw(x, d)
  if x == nil then return d end
  if type(x) == "boolean" then return x end
  return finNum(x, d and 1 or 0) >= 0.5
end

-- deep copy of plain tables (metatables are not copied)
local function copy(t)
  if type(t) ~= "table" then return t end
  local o = {}
  for k, x in pairs(t) do o[k] = copy(x) end
  return o
end

-- non-empty string, else nil
local function str(v)
  if type(v) == "string" and v ~= "" then
    return v
  end
  return nil
end

M.clamp = clamp
M.clampMinMax = clampMinMax
M.clamp01 = clamp01
M.clamp01NanZero = clamp01NanZero
M.smoothstep = smoothstep
M.flag = flag
M.num = num
M.finNum = finNum
M.fin = fin
M.lin = lin
M.sgn = sgn
M.sw = sw
M.copy = copy
M.str = str

return M
