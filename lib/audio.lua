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

-- MODULE DECLARATION ----------------------------------------------------------

local Audio = {
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Audio.__index = Audio

function Audio.new()
  local self = setmetatable({}, Audio)
  return self
end

-- LOCAL CONSTANTS -------------------------------------------------------------

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

function Audio:initialize(sounds)
  self.sounds = {}
  for id, sound in pairs(sounds) do
    local source = love.audio.newSource(sound.file, sound.mode or 'static')
    self.sounds[id] = {
        source = source,
        overlayed = sound.overlayed,
        looping = sound.looping,
        instances = {}
      }
  end
end

function Audio:deinitialize()
  self:halt()
  self.sounds = {}
end

function Audio:update(dt)
  -- For each sound source, find the inactive instances and remove them
  -- to release the resource.
  for _, sound in pairs(self.sounds) do
    local zombies = {}
    for index, instance in ipairs(sound.instances) do
      if not instance:isPlaying() then
        zombies[#zombies + 1] = index
      end
    end
    for _, index in ipairs(zombies) do
      table.remove(sound.instances, index)
    end
  end
end

function Audio:halt(sound_id)
  for id, sound in pairs(self.sounds) do
    -- Stop the selected sound (by id) instances, or if not specified halts
    -- the whole sounds bank.
    if not sound_id or sound_id == id then
      for _, instance in ipairs(sound.instances) do
        instance:stop()
      end
      sound.instances = {}
    end
  end
end

function Audio:play(id, volume)
  -- Retrieve the sound by id, and if not existing bail out.
  local sound = self.sounds[id]
  if not sound then
    return
  end

  -- Create a new source clone if the sound is multilayer or
  -- it's the first one of a "no-layered" one.
  if sound.overlayed or #sound.instances == 0 then
    local instance = sound.source:clone()
    instance:setVolume(volume or 1.0)
    instance:setLooping(sound.looping)

    sound.instances[#sound.instances + 1] = instance
  end

  -- In every case, the source to be controlled is the last
  -- instance.
  local instance = sound.instances[#sound.instances]

  -- If the sound instance is playing (i.e. it's a non-layered sound already
  -- playing) we rewind it to retrigger. Otherwise, we start it!
  if instance:isPlaying() then
    instance:rewind()
  else
    instance:play()
  end

  -- Return the audio source, so that it could be externally controlled.
  return instance
end

-- END OF MODULE ---------------------------------------------------------------

return Audio

-- END OF FILE -----------------------------------------------------------------
