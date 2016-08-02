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

local Input = {
  _VERSION = '0.2.0',
  keys = nil,
  accumulators = nil
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Input.__index = Input

function Input.new()
  local self = setmetatable({}, Input)
  return self
end

-- MODULE FUNCTIONS ------------------------------------------------------------

function Input:initialize(keys, groups)
  self.keys = {}
  self.accumulators = {}
  for id, group in pairs(keys) do
    self.keys[id] = { group = group, pressed = false }
    self.accumulators[group] = { delay = groups[group], time = 0 }
  end
end

function Input:update(dt)
  local keys = { amount = 0, pressed = {} }

  local counters = {}

  for id, key in pairs(self.keys) do
    local pressed = love.keyboard.isDown(id)

    if pressed then
      local accumulator = self.accumulators[key.group]

      -- Keep a group-based input counter, and increment the time
      -- accumulator if needed (i.e only on the first input)
      counters[key.group] = counters[key.group] and counters[key.group] + 1 or 1
      if counters[key.group] == 1 then
        accumulator.time = accumulator.time + dt
      end

      -- First key press or repeating key.
      if not key.pressed or accumulator.time >= accumulator.delay then
        keys.amount = keys.amount + 1
        keys.pressed[id] = true
      end
    end

    key.pressed = pressed
  end

  -- Reset the accumulators whose total time has passed the delay amount
  -- or has not been interacted with in the last iteration.
  for group, accumulator in pairs(self.accumulators) do
    if accumulator.time >= accumulator.delay or not counters[group] then
      accumulator.time = 0
    end
  end

  return keys
end

-- END OF MODULE ---------------------------------------------------------------

return Input

-- END OF FILE -----------------------------------------------------------------
