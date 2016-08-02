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
local constants = require('game.constants')
local Audio = require('lib.audio')
local graphics = require('lib.graphics')
local Tweener = require('lib.tweener')

-- MODULE DECLARATION ----------------------------------------------------------

local game = {
  world = require('game.world'),
}

-- MODULE FUNCTIONS ------------------------------------------------------------

function game:initialize(environment)
  self.environment = environment

  self.tweener = Tweener.new()
  self.tweener:initialize()

  self.audio = Audio.new()
  self.audio:initialize({
--      ['bgm'] = { file = 'assets/sounds/bgm.ogg', overlayed = false, looping = true, mode = 'stream' }
    })

  self.world:initialize()
end

function game:enter()
  self.world:reset()

  self.tweener:reset()
  
  -- Start the background music and create a tweener to fade in both the
  -- graphics and the audio.
--  self.audio:play('bgm', 1.0)
  self.tweener:linear(config.game.timeouts.music_in,
      function(ratio)
        love.audio.setVolume(ratio)
      end)

  --
  self.sequence = {
    { 'fade-in', config.game.timeouts.fade_in, 255, function(ratio)
          self.alpha = math.floor((1 - ratio) * 255)
        end },
    { 'intro', config.game.timeouts.intro, 255, nil },
    { 'intro-out', config.game.timeouts.intro_out, 191, function(ratio)
          self.alpha = math.floor((1 - ratio) * 191)
        end },
    { 'running', 0, 255, nil }
  }
  self.sequence_index = 0
  self.sequence_move_to_next = true
  
  -- Initially we a in no particular state.
  self.state = nil
end

function game:leave()
  self.audio:halt()
end

function game:input(keys, dt)
  -- We pass a nil input if we are not ready.
  if self.state == 'running' then
    self.world:input(keys, dt)
  elseif self.state == 'done' then
    if keys.pressed['z'] then
      self.world:reset()
      self.state = 'running'
    end
  else
    self.world:input(nil, dt)
  end
end

function game:update(dt)
  self.tweener:update(dt)

  self.audio:update(dt)

  if self.state == 'running' then
    self.world:update(dt)
  end

  -- Process the sequence of tweeners, untils it's finished!
  if self.sequence_move_to_next and self.sequence_index < #self.sequence then
    self.sequence_index = self.sequence_index + 1
    local state, timeout, alpha, on_update = unpack(self.sequence[self.sequence_index])
    self.state = state
    self.alpha = alpha
    self.sequence_move_to_next = false
    self.tweener:linear(timeout,
        function(ratio)
          if on_update then
            on_update(ratio)
          end
        end,
        function()
          self.sequence_move_to_next = true
        end)
  end

  -- If the world has come to an end, just fade out.
  if self.world:is_finished() then
    self.state = 'done'
  end
  
  return nil
end

function game:draw()
  -- At the very first iteration, we haven't set the state so we don't draw
  -- anything.
  if not self.state then
    return
  end
  
  self.world:draw()

  if self.state == 'fade-in' or self.state == 'intro' then
    graphics.fill('black', 191)
    graphics.text('DARN!',
      constants.SCREEN_RECT, 'retro-computer', 'white', 'center', 'middle', 2, 191)
  elseif self.state == 'intro-out' then
    graphics.fill('black', self.alpha)
    graphics.text('DARN!',
      constants.SCREEN_RECT, 'retro-computer', 'white', 'center', 'middle', 2, self.alpha)
  elseif self.state == 'done' then
    graphics.fill('black', 127)
    graphics.text('DARN!',
      constants.SCREEN_RECT, 'retro-computer', 'white', 'center', 'middle', 2)
    graphics.text('PRESS Z TO RESTART',
      constants.SCREEN_RECT, 'retro-computer', 'white', 'center', 'bottom', 1)
  end

  -- Fade the display if needed, by overlapping a translucent black rectangle.
  if self.state == 'fade-in' then
    graphics.fill('black', self.alpha)
  elseif self.state == 'intro' then
  elseif self.state == 'intro-out' then
  elseif self.state == 'running' then
  elseif self.state == 'done' then
  end

  love.graphics.setColor(255, 255, 255)
end

-- END OF MODULE ---------------------------------------------------------------

return game

-- END OF FILE -----------------------------------------------------------------
