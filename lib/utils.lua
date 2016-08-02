--[[

Copyright (c) 2016 by Marco Lizza (marco.lizza@gmail.com)

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

]]--

-- MODULE DECLARATION ----------------------------------------------------------

local utils = {
  _VERSION = '0.1.0'
}

-- LOCAL CONSTANTS -------------------------------------------------------------

-- The day is divided into five regions of time, that is "night", "morning",
-- "noon", "afternoon", and "evening" according to the following diagram.
--
--           1         2
-- 012345678901234567890123
-- |       |   |   |   |
-- nnnnnnnnMMMMNNNNAAAAEEEE
local SECONDS_IN_MINUTE = 60
local SECONDS_IN_HOUR = 60 * SECONDS_IN_MINUTE
local SECONDS_IN_DAY = 24 * SECONDS_IN_HOUR

local MORNING = 8 * SECONDS_IN_HOUR
local NOON = 12 * SECONDS_IN_HOUR
local AFTERNOON = 16 * SECONDS_IN_HOUR
local EVENING = 20 * SECONDS_IN_HOUR

local CONVERSION_TABLE = {
  { scale = SECONDS_IN_DAY, description = 'day' },
  { scale = SECONDS_IN_HOUR, description = 'hour' },
  { scale = SECONDS_IN_MINUTE, description = 'minute' },
  { scale = 1, description = 'second' }
}

-- MODULE FUNCTIONS ------------------------------------------------------------

-- Traslate a time to a printable format expression (e.g. '2 hours').
function utils.time_to_string(time)
  local result = {}

  local remainder = math.floor(time)

  for _, v in ipairs(CONVERSION_TABLE) do
    if remainder >= v.scale then
      local units = remainder / v.scale
      result[#result + 1] = string.format('%s %s%s', units, v.description, (units > 1 and 's' or ''))
      remainder = remainder % v.scale
    end
  end
  
  return table.concat(result, ' and ')
end

-- Formats the current game time (in floating-point seconds) to a more
-- conveniente "HH:MM:SS" format.
function utils.format_time(time, use_seconds)
  time = math.floor(time)
  local seconds = time % 60
  
  time = math.floor(time / 60)
  local minutes = time % 60
  
  time = math.floor(time / 60)
  local hours = time % 24

  if use_seconds then
  else
  end
  return string.format('%02d:%02d', hours, minutes)
end

--
function utils.to_days(time)
  return math.floor(time / SECONDS_IN_DAY)
end

-- Returns a string decribing the current time of day (e.g. "noon") given
-- the time expressed in seconds.
function utils.time_of_day(time)
  local hour = math.floor(time % SECONDS_IN_DAY)

  if hour >= MORNING and hour < NOON then
    return 'morning'
  elseif hour >= NOON and hour < AFTERNOON then
    return 'noon'
  elseif hour >= AFTERNOON and hour < EVENING then
    return 'afternoon'
  elseif hour >= EVENING then
    return 'evening'
  else
    return 'night'
  end
end

-- Scan the passed list of enabled virtual keys and returns both the current
-- pressed/release state snapshot. Also, return whether is there any pressed
-- key in the current scan.
function utils.grab_input(enabled)
  local keys = {}

  local has_input = false

  for _, id in ipairs(enabled) do
    keys[id] = love.keyboard.isDown(id)
    if keys[id] then
      has_input = true
    end
  end

  return keys, has_input
end

-- Quick and dirty color interpolation function. Work either with tables and
-- with numeric values.
function utils.lerp(from, to, alpha)
  if type(from) == 'table' then
    local result = {}
    for index, value in ipairs(from) do
      result[#result + 1] = utils.lerp(value, to[index], alpha)
    end
    return result
  else
    return (to - from) * alpha + from
  end
end

-- Returns a scaled (i.e. multiplied) version of a table. It also works with
-- numeric values, although is not so useful.
function utils.scale(values, factor)
  if type(values) == 'table' then
    local result = {}
    for _, value in ipairs(values) do
      result[#result + 1] = utils.scale(value, factor)
    end
    return result
  else
    return values * factor
  end
end

-- We are generically calling the source image "sheet" since it contains
-- the whole set of sub-images, and the quad-set "atlas" since it does
-- not contains any image-data but only rectangles used to pick the
-- frames from the sheet.
function utils.load_atlas(filename, frame_width, frame_height)
  local sheet = love.graphics.newImage(filename)
  local atlas = {}

  -- The frames are organized in the sheet in row-by-colums fashion,
  -- so we can get dynamically the amount of tiles with a couple of
  -- simple division.
  local columns = sheet:getWidth() / frame_width
  local rows = sheet:getHeight() / frame_height
  for i = 1, rows do
    for j = 1, columns do
      atlas[#atlas + 1] = love.graphics.newQuad((j - 1) * frame_width, (i - 1) * frame_height,
        frame_width, frame_height, sheet:getWidth(), sheet:getHeight())
    end
  end
  
  return sheet, atlas
end

-- Returns the sign of a given value.
function utils.sign(value)
  if value < 0 then
    return -1
  elseif value > 0 then
    return 1
  else
    return 0
  end
end

function utils.delta(a, b, c, d)
  local dx, dy
  if type(a) == 'table' and type(b) == 'table' then
    local ax, ay = unpack(a)
    local bx, by = unpack(b)
    dx, dy = ax - bx, ay - by
  else
    dx, dy = a - c, b - d
  end
  return dx, dy
end

function utils.distance(a, b, c, d)
  local dx, dy = utils.delta(a, b, c, d)
  return math.sqrt(dx * dx + dy * dy)
end

function utils.overlap(a, b, c, d)
  local ax, ay, bx, by = a, b, c, d
  if type(a) == 'table' and type(b) == 'table' then
    ax, ay = unpack(a)
    bx, by = unpack(b)
  end
  return ax == bx and ay == by
end

function utils.forward(index, table)
  return (index % #table) + 1
end

function utils.backward(index, table)
  return utils.forward(index + (#table - 2), table)
end

function utils.to_radians(angle)
  return angle * 0.01745329251994329576923690768489
end

function utils.to_degrees(angle)
  return angle * 57.295779513082320876798154814105
end

function utils.angle_to(from, to)
  local fx, fy = unpack(from)
  local tx, ty = unpack(to)
  return math.atan2(ty - fy, tx - fx)
end

function utils.clear(t, deep)
  for k, v in pairs(t) do
    if type(v) == 'table' and deep then
      utils.clear(v, deep)
    end
    t[k] = nil
  end
end

-- END OF MODULE ---------------------------------------------------------------

return utils

-- END OF FILE -----------------------------------------------------------------
