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

local Animation = {
  _VERSION = '0.1.0'
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Animation.__index = Animation

function Animation.new()
  local self = setmetatable({}, Animation)
  return self
end

-- MODULE FUNCTIONS ------------------------------------------------------------

function Animation:initialize(image, width, height, offset, count)
  offset = offset or 0
  count = count or math.huge

  -- The frames are organized in the sheet in row-by-colums fashion,
  -- so we can get dynamically the amount of tiles with a couple of
  -- simple division.
  local columns = math.floor(image:getWidth() / width)
  local rows = math.floor(image:getHeight() / height)
  local quads = {}
  for i = 1, rows do
    for j = 1, columns do
      if offset <= 0 then
        quads[#quads + 1] = love.graphics.newQuad((j - 1) * width, (i - 1) * height,
          width, height, image:getWidth(), image:getHeight())
      end

      offset = offset - 1

      count = count - 1
      if count <= 0 then
        break
      end
    end
  end
  
  self.image = image
  self.quads = quads
  
  self.period = 1 / 50
  self.on_loop = nil
  self.running = true
  
  self.index = 1
  self.elapsed = 0
end

function Animation:configure(frequency, on_loop)
  self.period = frequency and (1 / frequency) or self.period
  self.duration = #self.quads * self.period
  self.on_loop = on_loop
end

function Animation:update(dt)
  if not self.running then
    return
  end
  
  self.elapsed = self.elapsed + dt
  if self.elapsed >= self.duration then
    local loops = math.floor(self.elapsed / self.duration)
    if self.on_loop and loops > 0 then
      self.on_loop(self, loops)
    end
    self.elapsed = self.elapsed - (loops * self.duration)
  end
  
  if self.running then
    self.index = math.floor(self.elapsed / self.period) + 1
  end
end

function Animation:draw(...)
  local quad = self.quads[self.index]
  love.graphics.draw(self.image, quad, ...)
end

function Animation:pause()
  self.running = false
end

function Animation:resume()
  self.running = true
end

function Animation:rewind()
  self:seek(1)
end

function Animation:seek(index)
  -- Negative value indicates that we are indexing backward from the end of
  -- the sequence.
  if index < 0 then
    index = #self.quads - index + 1
  end
  self.index = index

  -- Clear the animation timer when the frame is forced to change, since we
  -- want the frequency to be respected.
  self.elapsed = 0
end

-- END OF MODULE ---------------------------------------------------------------

return Animation

-- END OF FILE -----------------------------------------------------------------
