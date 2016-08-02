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

local Animation = require('lib.animation')

-- MODULE DECLARATION ----------------------------------------------------------

local Animator = {
  _VERSION = '0.1.0'
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Animator.__index = Animator

function Animator.new()
  local self = setmetatable({}, Animator)
  return self
end

-- LOCAL CONSTANTS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

--{
--  defaults = {
--    width = 16,
--    height = 16,
--    frequency = 6,
--    on_loop = nil,
--  },
--  sequences = {
--    ['running'] = { filename = 'assets/image.png', offset = 0, count = 2 }
--  }
--}
function Animator:initialize(parameters)
  local images = {}

  local defaults = parameters.defaults
  
  local animations = {}
  for id, sequence in pairs(parameters.sequences) do
    -- Search in the images' cache for the specified image. If not present,
    -- load and update the cache content.
    local image = images[sequence.filename]
    if not image then
      image = love.graphics.newImage(sequence.filename)
      images[sequence.filename] = image
    end

    -- Create the animation and configure it.
    local animation = Animation.new()
    animation:initialize(image, sequence.width or defaults.width, sequence.height or defaults.height,
        sequence.offset, sequence.count)
    animation:configure(sequence.frequency or defaults.frequency, sequence.on_loop or defaults.on_loop)

    animations[id] = animation
  end

  self.images = images
  self.animations = animations
  self.current = nil
end

function Animator:update(dt)
  self.current:update(dt)
end

function Animator:draw(...)
  self.current:draw(...)
end

function Animator:switch_to(index, sync)
  if self.animations[index] then
    local animation = self.animations[index]
    if self.current ~= animation then
      animation:rewind()
      if sync then
        animation:seek(self.current.index)
      end
      self.current = animation
    end
  end
end

function Animator:pause()
  self.current:pause()
end

function Animator:resume()
  self.current:resume()
end

function Animator:rewind()
  self.current:rewind()
end

function Animator:seek(frame)
  self.current:seek(frame)
end

-- END OF MODULE ---------------------------------------------------------------

return Animator

-- END OF FILE -----------------------------------------------------------------
