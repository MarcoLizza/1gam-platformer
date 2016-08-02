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

local config = require('game.config')
local Animated = require('game.entities.animated')
local Static = require('game.entities.static')
local Hud = require('game.hud')

local Entities = require('lib.entities')
local Shaker = require('lib.shaker')
local Tweener = require('lib.tweener')

-- MODULE DECLARATION ----------------------------------------------------------

local world = {
}

-- LOCAL CONSTANTS -------------------------------------------------------------

local LIMBS = {
  { id = 'left-leg', damage = 0.05, tuning = 0.05, filename = 'assets/data/limb-left-leg.png' },
  { id = 'right-leg', damage = 0.05, tuning = 0.05, filename = 'assets/data/limb-right-leg.png' },
  { id = 'left-punch', damage = 0.03, tuning = 0.07, filename = 'assets/data/limb-left-punch.png' },
  { id = 'right-punch', damage = 0.03, tuning = 0.07, filename = 'assets/data/limb-right-punch.png' },
  { id = 'head', damage = 0.01, tuning = 0.03, filename = 'assets/data/limb-head.png' }
}

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- Computes the change in the range [0, 1] for the television to go off-synch
-- give the current tuning factor (in the range [0, 1])
function compute_chance(tuning)
  if tuning < 0.75 then
    local chance = 1 - (tuning / 0.75)
    local roll = love.math.random()
    local occurred = roll < chance
    return chance, occurred
  end
  return 0, false
end

-- MODULE FUNCTIONS ------------------------------------------------------------

function world:initialize()
  self.entities = Entities.new()
  self.entities:initialize(function(a, b) -- lower priority => earlier drawn
        return a.priority < b.priority
      end)

  self.shaker = Shaker.new()
  self.shaker:initialize()
  
  self.tweener = Tweener.new()
  self.tweener:initialize()

  self.hud = Hud.new()
  self.hud:initialize(self)
end

function world:reset()
  -- Reset the world "age" to zero. Also, pick the first scene as the current
  -- one and clear the "next" scene reference (will be detected automatically
  -- depending on the age)
  self.damage = 0.0
  self.tuning = 1.0
  self.would_interact = false

  -- Reset the entity manager and add the the player one at the center of the
  -- screen.
  self.entities:reset()
  self:setup()

  -- Reset the camera shaker to default state.
  self.shaker:reset()

  -- Reset the HUD state, too.
  self.hud:reset()
end

function world:input(keys, dt)
  if not keys then
    return
  end
  
  local would_interact = keys.pressed['x']

  local limb = self.entities:find(function(entity)
        return entity.id == 'limb'
      end)
  
  local can_interact = not limb
  
  if would_interact and can_interact then
    self:interact()
  end
end

function world:update(dt)
  -- After the interaction, check if the damage was too much. If the television
  -- is not damaged beyond repair, handle the tuning and damage.
  self.wrecked = self.damage >= 1.0
  
  if not self.wrecked then
    -- We are decreasing the tuning by a costant factor over time. When below
    -- a certaing threshold the television could go out of synch.
    self.tuning = math.max(0.0, self.tuning - 0.03 * dt)

    local chance, occurred = compute_chance(self.tuning)
    if chance == 0.0 then
      self:switch_to('display')
    elseif chance > 0.0 and occurred then
      self:switch_to('static')
    end

    -- We also decrease the current damage. Will reset to zero over time.
    self.damage = math.max(0.0, self.damage - 0.01 * dt)
  end

  -- Handle the submodules updates.
  self.entities:update(dt)
  self.shaker:update(dt)
  self.tweener:update(dt)

  self.hud:update(dt)
end

function world:draw()
  self.shaker:pre()
    self.entities:draw()
  self.shaker:post()
  
  self.hud:draw()
end

function world:is_finished()
  return self.damage >= 1.0 or self.tuning <= 0
end

function world:switch_to(index)
  local screen = self.entities:find(function(entity)
        return entity.id == 'screen'
      end)
  
  screen:switch_to(index)
end

function world:setup()
  local background = Static.new()
  background:initialize({
        id = 'background',
        priority = 0,
        life = math.huge,
        image = 'assets/data/background.png',
      })
  self.entities:push(background)

  local screen = Animated.new()
  screen:initialize({
        id = 'screen',
        priority = 1,
        life = math.huge,
        animations = {
          width = 480,
          height = 300,
          frequency = 4, -- 250ms per frame
          on_loop = nil,
          sequences = {
            ['static'] = 'assets/data/static.png',
            ['display'] = 'assets/data/display.png'
          },
          default = 'display'
        }
      })
  self.entities:push(screen)

  local television = Static.new()
  television:initialize({
        id = 'television',
        priority = 2,
        life = math.huge,
        image = 'assets/data/television.png'
      })
  self.entities:push(television)
end

function world:interact()
  local params = LIMBS[love.math.random(#LIMBS)] -- get a random limb
  
  local limb = Static.new()
  limb:initialize({
        id = 'limb',
        priority = 3,
        life = config.game.timeouts.limb,
        image = params.filename,
      })
  self.entities:push(limb)
  
  self.damage = math.min(1.0, self.damage + params.damage)
  self.tuning = math.min(1.0, self.tuning + params.tuning)
  
  self.shaker:add(params.damage * 100)
end

-- END OF MODULE -------------------------------------------------------------

return world

-- END OF FILE ---------------------------------------------------------------
