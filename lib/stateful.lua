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

local Stateful = {
  _VERSION = '0.2.0',
  states = {},
  current = nil
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Stateful.__index = Stateful

-- Create a new [Stateful] class instance.
function Stateful.new()
  local self = setmetatable({}, Stateful)
  return self
end

-- MODULE FUNCTIONS ------------------------------------------------------------

-- Initialized the instance, binding the passed states table and initializing
-- each state in sequence. The states can be indexed both by a numeric index
-- or table key. Along with the states list, we are passing also a table to
-- be used as a shared environment" (it will be propagated to the states, as
-- well).
function Stateful:initialize(states, environment)
  self.environment = environment
  
  for key, state in pairs(states) do
    state:initialize(self.environment)
    self.states[key] = state
  end
end

-- Leave the current state, if any, and switch to the new one (whose index/key
-- match the formal argument [key]).
function Stateful:switch_to(key)
  -- Check if the requested state exists. If not, bail out.
  local state = self.states[key]
  if not state then
    return
  end
  if self.current then
    self.current:leave()
  end
  self.current = state
  self.current:enter()
end

-- Handles the input from the user.
function Stateful:input(keys, dt)
  -- Current state not defined, bail out.
  if not self.current then
    return
  end
  self.current:input(keys, dt)
end

-- Update the current state, if any. Also handle the state-switch mechanism.
function Stateful:update(dt)
  -- Current state not defined, bail out.
  if not self.current then
    return
  end
  -- Update the current state. The result value is the key of the new state to
  -- be activated, or [nil] to continue with the current one.
  local key = self.current:update(dt)
  if key then
    self:switch_to(key)
  end
end

-- Draw the current state, if any.
function Stateful:draw()
  if not self.current then
    return
  end
  self.current:draw()
end

-- END OF MODULE ---------------------------------------------------------------

return Stateful

-- END OF FILE -----------------------------------------------------------------
