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

local soop = require('lib.soop')
local utils = require('lib.utils')

-- MODULE DECLARATION ----------------------------------------------------------

local Entity = soop.class()

-- MODULE FUNCTION -------------------------------------------------------------

function Entity:initialize(parameters)
  self.id = parameters.id -- can be [nil] for unnamed entities
  self.position = parameters.position or { 0, 0 }
  self.angle = parameters.angle or 0
  self.life = parameters.life or math.huge -- if not provided, entity will be indestructable
  self.radius = parameters.radius
end

function Entity:update(dt)
  -- Decrease the current life if the entity is not everlasting.
  if self.life ~= math.huge then
    self.life = math.max(0, self.life - dt)
  end
end

function Entity:hit(damage)
  damage = damage or 1
  if self.life ~= math.huge and self.life > 0 then
    self.life = math.max(0, self.life - damage)
  end
end

function Entity:kill()
  self.life = 0
end

function Entity:is_alive()
  return self.life > 0
end

function Entity:aabb()
  local x, y = unpack(self.position)
  return { x - self.radius, y - self.radius, x + self.radius, y + self.radius }
end

function Entity:collide(other)
  local distance = utils.distance(self.position, other.position)
  return distance <= (self.radius + other.radius)
end

function Entity:cast(modulo)
  local vx, vy = math.cos(self.angle) * modulo, math.sin(self.angle) * modulo
  local cx, cy = unpack(self.position)
  return cx + vx, cy + vy
end

function Entity:rotate(delta)
  self.angle = self.angle + delta
end

-- END OF MODULE ---------------------------------------------------------------

return Entity

-- END OF FILE -----------------------------------------------------------------
