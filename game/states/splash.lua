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

local graphics = require('lib.graphics')
local easing = require('lib.easing')

-- MODULE DECLARATION ----------------------------------------------------------

local splash = {
  states = {
    { mode = 'fade-in', delay = 1, file = 'assets/images/1gam.png' },
    { mode = 'display', delay = 5, file = 'assets/images/1gam.png' },
    { mode = 'cross-out', delay = 0.25, file = 'assets/images/1gam.png' },
    { mode = 'cross-in', delay = 0.25, file = 'assets/images/love2d.png' },
    { mode = 'display', delay = 5, file = 'assets/images/love2d.png' },
    { mode = 'fade-out', delay = 0.25, file = 'assets/images/love2d.png' }
  }
}

-- MODULE FUNCTIONS ------------------------------------------------------------

function splash:initialize()
end

function splash:enter()
  self.index = nil
end

function splash:leave()
  -- Release the image resource upon state leaving.
  self.image = nil
end

function splash:input(keys, dt)
end

function splash:update(dt)
  -- Determine if we should move to the next state. This happens if the index is
  -- not defined or if (after advancing the progress counter) the timeout has
  -- elapsed.
  local change = false
  
  if not self.index then
    self.index = 0
    change = true
  elseif self.progress < self.delay then
    self.progress = self.progress + dt
    if self.progress >= self.delay then
      change = true
    end
  end

  if change then
    -- Advance to the next state. When the end of the sequence is reached, we
    -- need to switch to the game state.
    self.index = self.index + 1
    if self.index > #self.states then
      return 'game'
    end

    -- Get the next state. If an image is defined, pre-load it. Then, we
    -- store the new state delay and reset the progress variable.
    local state = self.states[self.index]
    if state.file then
      self.image = love.graphics.newImage(state.file)
    else
      self.image = nil
    end
    self.delay = state.delay
    self.progress = 0
  end

  return nil
end

function splash:draw()
  -- If the state index has not been updated yet, skip and wait next
  -- iteration.
  if not self.index then
    return
  end
  
  -- If defined, draw the background image.
  if self.image then
    love.graphics.draw(self.image, 0, 0)
  end

  -- Calculate the current fading progress ratio.
  local alpha = self.progress / self.delay

  -- According to the current mode, compute the fading color.
  local color = nil
  local state = self.states[self.index]
  if state.mode == 'fade-in' then -- from black
    alpha = easing.quadratic(1.0 - alpha) * 255
    color ='black'
  elseif state.mode == 'fade-out' then -- to black
    alpha = easing.quadratic(alpha) * 255
    color ='black'
  elseif state.mode == 'cross-in' then -- from white
    alpha = easing.quadratic(1.0 - alpha) * 255
    color ='white'
  elseif state.mode == 'cross-out' then -- to white
    alpha = easing.quadratic(alpha) * 255
    color ='white'
  end

  -- If the overlay "fading" color is defined, draw a full size filled
  -- rectangle over the current display.
  if color then
    graphics.fill(color, alpha)
  end
  
  love.graphics.setColor(255, 255, 255)
end

-- END OF MODULE ---------------------------------------------------------------

return splash

-- END OF FILE -----------------------------------------------------------------
