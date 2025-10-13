/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Tan Kui An Andrew
 */

function hitTestIcon(icons, testI) {
  return hitTestSimple(icons, testI, 1.0);
}

function hitTestTextAnnotation(textAnnotations, testTA) {
  for (let ta of textAnnotations) {
    let fontSize = ta.size;
    let lines = ta.content ? ta.content.split('\n') : [];
    let height = fontSize * lines.length * 1.2;
    // TODO: improve hittest. refer tt manual layout.
    if (testTA.y >= ta.y && testTA.y <= ta.y + height) {
      if (sqDist([ta.x, ta.y], [testTA.x, ta.y]) <= 0.4) {
        return ta;
      }
    }
  }
  return false;
}

function hitTestSimple(objects, testObj, hitDistance) {
  for (let obj of objects) {
    if (obj === testObj || obj === testObj.ob) {
      continue;
    }
    if (hitTestSimpleBetweenItems(obj, testObj, hitDistance)) {
      return obj;
    }
  }
  return false;
}

function hitTestSimpleBetweenItems(item1, item2, hitDistance) {
  /* Return true if the distance between the 2 items are less than hitDistance, false if otherwise. */
  return sqDist([item1.x, item1.y], [item2.x, item2.y]) <= hitDistance * hitDistance - 0.001;
}

function sqDist(a, b) {
  var d0 = a[0] - b[0];
  var d1 = a[1] - b[1];
  return d0 * d0 + d1 * d1;
}

function hitTestRect(textAnnotations, iconAnnotations, testPt1, testPt2, tolerance) {
  /* Return list of objects inside rect */
  let objects = [];

  let maxX = Math.max(testPt1[0], testPt2[0]);
  let maxY = Math.max(testPt1[1], testPt2[1]);
  let minX = Math.min(testPt1[0], testPt2[0]);
  let minY = Math.min(testPt1[1], testPt2[1]);

  // TextAnnotations
  objects = textAnnotations.filter(
    (j) =>
      j.x >= minX - tolerance &&
      j.x <= maxX + tolerance &&
      j.y >= minY - tolerance &&
      j.y <= maxY + tolerance
  );

  // IconAnnotations
  objects = objects.concat(
    iconAnnotations.filter(
      (j) =>
        j.x >= minX - tolerance &&
        j.x <= maxX + tolerance &&
        j.y >= minY - tolerance &&
        j.y <= maxY + tolerance
    )
  );

  return objects;
}

export default {
  /* Hit Test */
  iconAnnotation: hitTestIcon,
  textAnnotation: hitTestTextAnnotation,
  rect: hitTestRect
};
