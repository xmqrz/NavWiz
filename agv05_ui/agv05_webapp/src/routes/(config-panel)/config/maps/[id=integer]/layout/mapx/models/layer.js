/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

const list = [
  ['LAYER_0', 0],
  ['LAYER_1', 1],
  ['LAYER_2', 2],
  ['LAYER_3', 3],
  ['LAYER_4', 4]
];

function fromEntries(entries) {
  return entries.reduce((obj, [key, value]) => {
    obj[key] = value;
    return obj;
  }, {});
}

/* Layer enum */

export default Object.assign(
  {
    LAYER_NA: list.length,
    list: list
  },
  fromEntries(list)
);
