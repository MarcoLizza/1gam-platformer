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

local Tweener = {
  _VERSION = '0.1.1'
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Tweener.__index = Tweener

function Tweener.new()
  local self = setmetatable({}, Tweener)
  return self
end

-- LOCAL CONSTANTS -------------------------------------------------------------

local EASING = {
  ['linear'] = function(value) return value end,
  ['quadratic'] = function(value) return math.pow(value, 2.0) end,
  ['cubic'] = function(value) return math.pow(value, 3.0) end,
  ['hill'] = function(value) return value > 0.5 and 1.0 or value * 2 end
}

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

function Tweener:initialize()
  self:reset()
end

function Tweener:reset()
  self.active = {}
  self.removed = {}
  self.incoming = {}
end

function Tweener:update(dt)
  while #self.incoming > 0 do
    local pair = table.remove(self.incoming)
    -- Unnamed closure will at the back be stored in the "indexed" part of
    -- the table.
    if pair.id then
      self.active[pair.id] = pair.closure
    else
      table.insert(self.active, pair.closure)
    end
  end
  
  local zombies = {}
  for id, closure in pairs(self.active) do
    local remove = self.removed[id] or closure(dt)
    if remove then
      zombies[#zombies + 1] = id
    end
  end

  for _, id in pairs(zombies) do
    self.active[id] = nil
  end
  self.removed = {}
end

function Tweener:remove(id)
  self.removed[id] = true
end

function Tweener:linear(time, callback, id)
  self:custom('linear', time, callback, id)
end

function Tweener:cubic(time, callback, id)
  self:custom('cubic', time, callback, id)
end

function Tweener:quadratic(time, callback, id)
  self:custom('quadratic', time, callback, id)
end

function Tweener:hill(time, callback, id)
  self:custom('hill', time, callback, id)
end

function Tweener:custom(mode, time, on_update, on_complete, id)
  local current = 0
  local easing = EASING[mode]
  -- Create a new tweener function (as a "closure"). The tweener will be
  -- removed from the list either in one of the following cases
  --   i) the tweening time is elapsed, or
  --   ii) the callback function returns a non-nil value, or
  --   iii) the [Tween:remove] is called from somewhere.
  closure = function(dt)
        current = current + dt
        if current > time then
          current = time
        end
        local completed = current == time
        local ratio = current / time
        local remove = on_update(easing(ratio))
        if completed and on_complete then
          on_complete()
        end
        return remove or completed
      end
  -- If provided, store the new tweener using the [id] as a key. Otherwise, we
  -- consider it "unnamed" and just store at the back of the integer-indexed
  -- part of the table.
  table.insert(self.incoming, {
        id = id,
        closure = closure
      })
end

-- END OF MODULE ---------------------------------------------------------------

return Tweener

-- END OF FILE -----------------------------------------------------------------
