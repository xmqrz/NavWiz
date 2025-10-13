/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default function (scene) {
  /* Terminals */
  var terminals = scene.append('g');
  terminals.attr('class', 'terminals');

  function modelsUpdated(data) {
    terminals.selectAll('g').remove();

    var t = terminals.selectAll('g').data(data).join('g');
    t.attr('id', (d) => d.id).attr('class', 'node');
    t.append('title').text((d) => d.title);
    t.append('rect')
      .attr('rx', 15)
      .attr('fill', (d) => (d.active ? 'gray' : 'white'))
      .attr('stroke', 'black')
      .attr('stroke-width', 1)
      .attr('width', (d) => d.width)
      .attr('height', (d) => d.height)
      .attr('x', (d) => d.x - d.width / 2)
      .attr('y', (d) => d.y - d.height / 2);
    t.append('text')
      .text((d) => d.name)
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y)
      .attr('font-weight', 'bold')
      .attr('font-size', 18)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'middle')
      .attr('dominant-baseline', 'middle');
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
