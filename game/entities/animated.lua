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

local Entity = require('game.entities.entity')
local Animator = require('lib.animator')
local soop = require('lib.soop')

-- MODULE DECLARATION ----------------------------------------------------------

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

local Animated = soop.class(Entity)

-- LOCAL CONSTANTS -------------------------------------------------------------

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

function Animated:initialize(parameters)
  Entity.initialize(self, parameters)

  self.type = 'animated'
  self.priority = parameters.priority
  self.reference = self.life

  local animations = {
    defaults = {
      width = parameters.animations.width,
      height = parameters.animations.height,
      frequency = parameters.animations.frequency,
      on_loop = parameters.animations.on_loop
    },
    sequences = { }
  }
  for id, filename in pairs(parameters.animations.sequences) do
    animations.sequences[id] = { filename = filename }
  end

  self.animator = Animator.new()
  self.animator:initialize(animations)
  self.animator:switch_to(parameters.animations.default)
end

function Animated:update(dt)
  Entity.update(self, dt)

  self.animator:update(dt)
end

function Animated:draw()
  -- Making sure not to colorize the sprite!
  love.graphics.setColor(255, 255, 255)

  local x, y = unpack(self.position)
  self.animator:draw(x, y)
end

function Animated:switch_to(index)
  -- If the index is the current active, no change will be performed and the
  -- current animation will keep on rolling.
  self.animator:switch_to(index)
end

-- END OF MODULE ---------------------------------------------------------------

return Animated

-- END OF FILE -----------------------------------------------------------------
