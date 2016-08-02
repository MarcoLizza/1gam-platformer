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

-- MODULE INCLUSIONS -----------------------------------------------------------

-- MODULE DECLARATION ----------------------------------------------------------

local Shaker = {
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Shaker.__index = Shaker

function Shaker.new()
  local self = setmetatable({}, Shaker)
  return self
end

-- LOCAL CONSTANTS -------------------------------------------------------------

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

function Shaker:initialize(frequency, dampening, amplitude, cap)
  self.frequency = frequency or 100
  self.dampening = dampening or 10
  self.amplitude = amplitude or 3
  self.cap = cap or 7
  self:reset()
end

function Shaker:reset()
  self.amount = 1
  self.time = 0
end

function Shaker:add(amount)
  self.amount = math.min(self.cap, self.amount + amount)
end

function Shaker:update(dt)
  self.amount = math.max(1, self.amount - self.dampening * dt)
  self.time = self.time + dt
end

function Shaker:pre()
  local factor = math.log(self.amount) * self.amplitude

  local angle = self.time * self.frequency
  local x = math.sin(angle) * factor
  local y = math.cos(angle) * factor

  love.graphics.push()
  love.graphics.translate(math.floor(x), math.floor(y))
end

function Shaker:post()
  love.graphics.pop()
end

-- END OF MODULE ---------------------------------------------------------------

return Shaker

-- END OF FILE -----------------------------------------------------------------
