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

local Entities = {
  _VERSION = '0.2.0'
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Entities.__index = Entities

function Entities.new()
  local self = setmetatable({}, Entities)
  return self
end

-- LOCAL FUNCTIONS -------------------------------------------------------------

-- MODULE FUNCTIONS ------------------------------------------------------------

function Entities:initialize(comparator, grid_size)
  -- Store the entity sorting-comparator (optional).
  self.comparator = comparator
  self.grid_size = grid_size

  self:reset()
end

function Entities:reset()
  self.active = {}
  self.incoming = {}
  self.colliding = {}
end

function Entities:update(dt)
  -- If there are any waiting recently added entities, we merge them in the
  -- active entities list. The active list is kept sorted, if a proper
  -- comparator was provided.
  if #self.incoming > 0 then
    for _, entity in ipairs(self.incoming) do
      table.insert(self.active, entity);
    end
    self.incoming = {}
    if self.comparator then
      table.sort(self.active, self.comparator)
    end
  end
  -- Update and keep track of the entities that need to be removed.
  --
  -- Since we need to keep the entities relative sorting, we remove "dead"
  -- entities from the back to front. To achive this we "push" the
  -- indices at the front of the to-be-removed list. That way, when
  -- we traverse it we can safely remove the elements as we go.
  local zombies = {}
  for index, entity in ipairs(self.active) do
    entity:update(dt)
    if not entity:is_alive() then
      table.insert(zombies, 1, index);
    end
  end
  for _, index in ipairs(zombies) do
    table.remove(self.active, index)
  end

  -- Keep the [colliding] attribute updated with the collision
  -- list.
  self.colliding = {}
  if self.grid_size then
    local grid = self:partition(self.grid_size)
    for _, entities in pairs(grid) do
      self:resolve(entities, self.colliding)
    end
  end
end

function Entities:draw()
  for _, entity in pairs(self.active) do
    entity:draw()
  end
end

function Entities:push(entity)
  -- We store thre entity-manager reference in the entity itself. Could be
  -- useful.
  entity.entities = self

  -- We enqueue the added entries in a temporary list. Then, in the "update"
  -- function we merge the entries with the active entries list and sort it.
  -- 
  -- The rationale for this is to decouple the entities scan/iteration and
  -- the addition. For example, it could happen that during an iteration we
  -- add new entities; however we cannot modify the active entities list
  -- content while we iterate.
  --
  -- We are using the "table" namespace functions since we are possibly
  -- continously scambling the content by reordering it.
  table.insert(self.incoming, entity)
end

function Entities:partition(size)
  local grid = {}
  
  for _, entity in ipairs(self.active) do
    -- If the entity does not have both the [collide] and [aabb] methods we
    -- consider it to be "ephemeral" in nature (e.g. sparkles, smoke, bubbles,
    -- etc...). It will be ignored and won't count toward collision.
    if entity.collide and entity.aabb then
      local aabb = entity.aabb()
      local left, top, right, bottom = unpack(aabb)
      local coords = {
            { left, top },
            { left, bottom },
            { right, top },
            { right, bottom }
          }

      -- We find the belonging grid-cell for each of the entity AABB corner,
      -- in order to deal with boundary-crossing entities. We make sure not
      -- to store the same entity twice in the same grid-cell.
      local cells = {}
      for _, position in ipairs(coords) do
        local x, y = unpack(position)
        local gx, gy = math.floor(x / size), math.floor(y / size)
        local id = string.format('%d@%d', gx, gy)
        if not grid[id] then
          grid[id] = {}
        end
        if not cells[id] then
          table.insert(grid[id], entity)
        end
        cells[id] = true
      end

      entity.cells = {}
      for id, _ in pairs(cells) do
        table.insert(entity.cells, id)
      end
    end
  end

  return grid
end

function Entities:resolve(entities, colliding)
  -- Naive bruteforce O(n^2) collision resolution algorithm (with no
  -- projection at all). As a minor optimization, we scan the pairing
  -- square matrix on the upper (or lower) triangle.
  --
  --     1 2 3 4
  --   1 . x x x
  --   2 . . x x
  --   3 . . . x
  --   4 . . . .
  --
  -- This needs "n(n-1)/2" checks.
  --
  -- http://buildnewgames.com/broad-phase-collision-detection/
  -- http://www.java-gaming.org/index.php?topic=29244.0
  -- http://www.hobbygamedev.com/adv/2d-platformer-advanced-collision-detection/
  -- http://www.wildbunny.co.uk/blog/2011/12/14/how-to-make-a-2d-platform-game-part-2-collision-detection/
  for i = 1, #entities - 1 do
    local this = entities[i]
    for j = i + 1, #entities do
      local that = entities[j]
      if this:collide(that) then
        colliding[#colliding + 1] = { this, that }
      end
    end
  end
end

function Entities:find(filter)
  for _, entity in ipairs(self.active) do
    if filter(entity) then
      return entity
    end
  end
  for _, entity in ipairs(self.incoming) do
    if filter(entity) then
      return entity
    end
  end
  return nil
end

-- END OF MODULE ---------------------------------------------------------------

return Entities

-- END OF FILE -----------------------------------------------------------------
