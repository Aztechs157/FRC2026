package org.team157.utilities;

import edu.wpi.first.math.Pair;
import java.util.Collection;
import java.util.HashMap;
import java.util.TreeMap;

/**
 * A mapping of values from K to V, sorted by a priority assigned to each value.
 *
 * <p>This largely functions like a {@link java.util.TreeMap TreeMap} but with a custom comparator
 * that uses priorities given by the user to order the entries. However, it also lets you use just
 * the key to access the value without needing to know the priority given to that key-value mapping.
 *
 * <p>Priorities are integers, where a lower number corresponds to an earlier value, and higher
 * numbers to a later values. When there is a tie the key itself is used as a tie-breaker to ensure
 * consistency when values are accessed.
 */
public final class PriorityMap<K extends Comparable<K>, V> {

  private TreeMap<Pair<Integer, K>, V> valueMap =
      new TreeMap<Pair<Integer, K>, V>(
          (p1, p2) -> {
            var pair1 = (Pair<Integer, K>) p1;
            var pair2 = (Pair<Integer, K>) p2;

            var i1 = pair1.getFirst().compareTo(pair2.getFirst());
            if (i1 != 0) {
              return i1;
            }

            return pair1.getSecond().compareTo(pair2.getSecond());
          });
  private HashMap<K, Integer> indexMap = new HashMap<K, Integer>();

  /**
   * Associates the specified value with the specified key at the specified priority.
   *
   * <p>Lower priorities come earlier than later priorities, when there is a tie the comparison
   * falls back to the key.
   *
   * @param key key with which the specified value is to be associated
   * @param priority the priority to associate with the key-value pair
   * @param value value to be associated with the specified key
   * @return the previous value associated with the key, or <code>null</code> if there was no
   *     mapping for the key. (A <code>null</code> return can also indicate that the map previously
   *     associated <code>null</code> with key.)
   */
  public V put(K key, int priority, V value) {
    var oldValue = valueMap.put(new Pair<Integer, K>(priority, key), value);
    indexMap.put(key, priority);

    return oldValue;
  }

  /**
   * Removes the mapping for this key from this <code>PriorityMap</code> if present.
   *
   * @param key key for which mapping should be removed
   * @return the previous value associated with key, or <code>null</code> if there was no mapping
   *     for key. (A <code>null</code> return can also indicate that the map previously associated
   *     <code>null</code> with key.)
   */
  public V remove(K key) {
    if (!indexMap.containsKey(key)) {
      return null;
    }

    var index = indexMap.remove(key);
    return valueMap.remove(new Pair<Integer, K>(index, key));
  }

  /**
   * Returns the first value according to the priorities given for each key-value mapping, or <code>
   * null</code> if the map is empty.
   *
   * <p>If there is a tie for the lowest priority, the key will be used as a tie-breaker.
   *
   * @return the value with the lowest priority, or <code>null</code> if the map is empty
   */
  public V firstValue() {
    if (isEmpty()) {
      return null;
    }

    return valueMap.firstEntry().getValue();
  }

  /**
   * Returns the last value according to the priorities given for each key-value mapping, or <code>
   * null</code> if the map is empty.
   *
   * <p>If there is a tie for the highest priority, the key will be used as a tie-breaker.
   *
   * @return the value with the highest priority, or <code>null</code> if the map is empty
   */
  public V lastValue() {
    if (isEmpty()) {
      return null;
    }

    return valueMap.lastEntry().getValue();
  }

  /**
   * Returns the first key according to the priorities given for each key-value mapping, or <code>
   * null</code> if the map is empty.
   *
   * <p>If there is a tie for the lowest priority, the key will be used as a tie-breaker.
   *
   * @return the key with the lowest priority, or <code>null</code> if the map is empty
   */
  public K firstKey() {
    if (isEmpty()) {
      return null;
    }

    return valueMap.firstEntry().getKey().getSecond();
  }

  /**
   * Returns the last key according to the priorities given for each key-value mapping, or <code>
   * null</code> if the map is empty.
   *
   * <p>If there is a tie for the highest priority, the key will be used as a tie-breaker.
   *
   * @return the key with the highest priority, or <code>null</code> if the map is empty
   */
  public K lastKey() {
    if (isEmpty()) {
      return null;
    }

    return valueMap.lastEntry().getKey().getSecond();
  }

  public Integer firstPriority() {
    if (isEmpty()) {
      return null;
    }
    return valueMap.firstEntry().getKey().getFirst();
  }

  public Integer lastPriority() {
    if (isEmpty()) {
      return null;
    }
    return valueMap.lastEntry().getKey().getFirst();
  }

  /**
   * Returns <code>true</code> if the map contains a mapping for the specified key
   *
   * @param key the key whose presence in this map is to be tested
   * @return <code>true</code> if this map contains a mapping for the specified key.
   */
  public boolean containsKey(K key) {
    return indexMap.containsKey(key);
  }

  /**
   * Returns the value to which the specified key is mapped, or <code>null</code> if this map
   * contains no mapping for the key.
   *
   * <p>A return value of <code>null</code> does not necessarily indicate that the map contains no
   * mapping for the key; it's also possible that the map explicitly maps the key to <code>null
   * </code>. The containsKey operation may be used to distinguish these two cases.
   *
   * @param key the key whose associated value is to be returned
   * @return the value to which the specified key is mapped, or <code>null</code> if this map
   *     contains no mapping for the key
   */
  public V get(K key) {
    if (!indexMap.containsKey(key)) {
      return null;
    }
    var index = indexMap.get(key);
    return valueMap.get(new Pair<Integer, K>(index, key));
  }

  /**
   * Returns the priority the given key is associated at.
   *
   * @param key the key whose priority is to be returned
   * @return the priority to which the specified key is mapped at, or <code>null</code> if this map
   *     contains no mapping for the key
   */
  public Integer getPriority(K key) {
    return indexMap.get(key);
  }

  /** Removes all of the mappings from this map. The map will be empty after this call returns. */
  public void clear() {
    indexMap.clear();
    valueMap.clear();
  }

  /**
   * Returns a shallow copy of this <code>PriorityMap</code> instance: the keys and values
   * themselves are not cloned.
   *
   * @return a shallow copy of this map
   */
  @SuppressWarnings("unchecked")
  public PriorityMap<K, V> clone() {
    var newPriorityMap = new PriorityMap<K, V>();

    newPriorityMap.indexMap = (HashMap<K, Integer>) indexMap.clone();
    newPriorityMap.valueMap = (TreeMap<Pair<Integer, K>, V>) valueMap.clone();

    return newPriorityMap;
  }

  /**
   * Return the number of key-value mappings in this map.
   *
   * @return the number of key-value mappings in this map
   */
  public int size() {
    return indexMap.size();
  }

  /**
   * Returns a Collection view of the values contained in this map.
   *
   * @return a collection view of the values contained in this map
   * @see java.util.TreeMap#values
   */
  public Collection<V> values() {
    return valueMap.values();
  }

  /**
   * Replaces the entry for the specified key only if it is currently mapped to some value, with the
   * same priority as the previous mapping.
   *
   * @param key key with which the specified value is associated
   * @param value value to be associated with the specified key
   * @return the previous value associated with the specified key, or <code>null</code> if there was
   *     no mapping for the key. (A <code>null</code> return can also indicate that the map
   *     previously associated null with the key, if the implementation supports <code>null</code>
   *     values.)
   */
  public V replace(K key, V value) {
    if (!containsKey(key)) {
      return null;
    }

    var index = indexMap.get(key);
    var oldValue = valueMap.replace(new Pair<Integer, K>(index, key), value);
    if (oldValue == null) {
      throw new NullPointerException();
    }

    return oldValue;
  }

  /**
   * Replaces the entry for the specified key at the specified priority only if it is currently
   * mapped to some value.
   *
   * @param key key with which the specified value is associated
   * @param priority the priority to associate with the key-value pair
   * @param value value to be associated with the specified key
   * @return the previous value associated with the specified key, or <code>null</code> if there was
   *     no mapping for the key. (A <code>null</code> return can also indicate that the map
   *     previously associated null with the key, if the implementation supports <code>null</code>
   *     values.)
   */
  public V replace(K key, int priority, V value) {
    if (!containsKey(key)) {
      return null;
    }

    var oldIndex = indexMap.remove(key);
    var oldValue = valueMap.remove(new Pair<Integer, K>(oldIndex, key));

    if (oldValue == null) {
      throw new AssertionError("priority map value is null when it shouldn't be null");
    }

    var oldValue2 = put(key, priority, value);

    if (oldValue2 != null) {
      throw new AssertionError("priority map value isn't null when it should be null");
    }

    return oldValue;
  }

  /**
   * Returns <code>true</code> if this map contains no key-value mappings.
   *
   * @return <code>true</code> if this map contains no key-value mappings
   */
  public boolean isEmpty() {
    return indexMap.isEmpty();
  }
}
