local collections = {
}

function collections.select(table, comparator)
  for key, value in pairs(table) do
    if comparator(key, value) then
      return value
    end
  end
  return nil
end

function collections.find(table, comparator)
  for key, value in pairs(table) do
    if comparator(key, value) then
      return key
    end
  end
  return nil
end

function collections.foreach(table, callback)
  for key, value in pairs(table) do
    callback(key, value)
  end
end

function collections.merge(target, source)
  for key, value in pairs(source) do
    if type(value) == "table" then
      local merged = target[key] or {}
      merge(merged, value)
      target[key] = merged
    else
      target[key] = value
    end
  end
end

return collections
