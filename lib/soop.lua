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

local soop = {
}

-- MODULE FUNCTION -------------------------------------------------------------

function soop.class(base)
  local proto = {}
  -- If a base class is defined, the copy all the functions.
  --
  -- This is an instant snapshot, any new field defined runtime in the base
  -- class won't be visible in the derived class.
  if base then
    soop.implement(proto, base)
  end
  -- This is the standard way in Lua to implement classes.
  proto.__index = proto
  proto.new = function(params)
      local self = setmetatable({}, proto)
      if self.__ctor then
        self.__ctor(params)
      end
      return self
    end
  return proto
end

function soop.implement(proto, model)
  for key, value in pairs(model) do
    if type(value) == 'function' then
      proto[key] = value
    end
  end
end

-- END OF MODULE ---------------------------------------------------------------

return soop

-- END OF FILE -----------------------------------------------------------------
