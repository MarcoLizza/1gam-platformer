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

local array = {}

-- MODULE FUNCTIONS ------------------------------------------------------------

function array.create(width, height, filler)
  local array = {}
  for y = 1, height do
    array[y] = {}
    for x = 1, width do
        array[y][x] = filler(x, y)
    end
  end
  return array
end

-- Durstenfeld 
function array.shuffle_in_place(array)
  for i = #array, 2, -1 do
    local j = love.math.random(i)
    array[i], array[j] = array[j], array[i]
  end
end

function array.shuffle(input)
  local output = {}
  for i = 1, #input do
    local j = love.math.random(i)
    if i ~= j then -- could be omitted
      output[i] = output[j]
    end
    output[j] = input[i]
  end
  return output
end

function array.contains(array, value)
  for _, v in ipairs(array) do
    if v == value then
      return true
    end
  end
  return false
end

function array.remove(array, value)
  for k, v in ipairs(array) do
    if v == value then
      table.remove(array, k)
    end
  end
end

-- END OF MODULE ---------------------------------------------------------------

return array

-- END OF FILE -----------------------------------------------------------------
