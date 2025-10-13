/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

export default function (scene) {
  /* Nodes */
  var nodes = scene.append('g');
  nodes.attr('class', 'nodes');

  function modelsUpdated(data) {
    // TODO: solve redrawing issue, each subelement need selectAll?.
    nodes.selectAll('g.node').remove();

    if (data.length <= 0) {
      return;
    }

    let a = nodes.selectAll('g.node').data(data).join('g').attr('class', 'node');

    // draw action
    let t = a
      .filter((d) => !d.drag)
      .attr('id', (d) => d.id)
      .style('cursor', (d) => d.cursor);
    t.append('title').text((d) => d.title);
    t.append('rect') // background rect
      .attr('fill', (d) => (d.active ? 'limegreen' : 'white'))
      .attr('rx', 15)
      .attr('stroke', 'black')
      .attr('stroke-width', 1)
      .attr('width', (d) => d.width)
      .attr('height', (d) => d.height)
      .attr('x', (d) => d.x - d.width / 2)
      .attr('y', (d) => d.y - d.height / 2)
      .style('filter', (d) => (d.float ? 'drop-shadow(2px 4px 6px black)' : ''));
    t.append('rect') // title rect
      .attr('stroke', 'black')
      .attr('stroke-width', 1)
      .attr('fill', (d) => d.color)
      .attr('width', (d) => d.width - 26)
      .attr('height', 31)
      .attr('x', (d) => d.x - d.width / 2 + 13)
      .attr('y', (d) => d.y - d.height / 2 + 13);
    t.append('text') // title text
      .text((d) => d.name)
      .attr('font-weight', 'bold')
      .attr('font-size', 16)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'middle')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y - d.height / 2 + 13 + 16);

    let l = t.filter((d) => d.url).append('a'); // taskTemplateUrl
    l.attr('xlink:href', (d) => d.url)
      .attr('xlink:title', 'Open nested task template')
      .attr('target', '_blank');
    l.append('text') // outcome text
      .text('\u27A6')
      .attr('font-weight', 'bold')
      .attr('font-size', 16)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'middle')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.x + d.width / 2 - 16 - 10)
      .attr('y', (d) => d.y - d.height / 2 + 13 + 16);

    // parameters

    t.append('rect') // param rect
      .attr('stroke', 'black')
      .attr('stroke-width', 1)
      .attr('fill', 'wheat')
      .attr('width', (d) => d.width - 26)
      .attr('height', (d) => d.height - 10 - 26 - 31 - 28)
      .attr('x', (d) => d.x - d.width / 2 + 13)
      .attr('y', (d) => d.y - d.height / 2 + 13 + 31 + 5);

    t.append('text') // param title text
      .text('Parameters:')
      .attr('font-weight', 'bold')
      .attr('font-size', 14)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'left')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.x - d.paramWidth / 2)
      .attr('y', (d) => d.y - d.height / 2 + 13 + 31 + 5 + 12);

    let p = t
      .selectAll('g.param')
      .data(function (d) {
        let x = d.x - d.paramWidth / 2;
        let baseY = d.y - d.height / 2 + 13 + 31 + 5 + 12 + 16;
        return d.paramsText.map(function (p, idx) {
          return {
            text: p,
            x: x,
            y: baseY + 16 * idx
          };
        });
      })
      .join('g');
    p.attr('class', 'param');
    p.append('text') // param text
      .text((d) => d.text)
      .attr('font-weight', 'normal')
      .attr('font-size', 14)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'left')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y)
      .style('white-space', 'pre');

    // outcomes
    let o = t
      .selectAll('g.outcome')
      .data((d) => d.outcomes)
      .join('g');
    o.attr('class', 'outcome edge').attr('id', (d) => d.id);
    let g = o
      .append('a') // to match with graphviz.
      // HACK: the "xlink:href" set here will become "href" attr causing nav event.
      // .attr('xlink:href', 'javascript:void(0)')
      .attr('class', 'cursor-pointer')
      .attr('xlink:title', (d) => `"${d.name}" Outcome`);
    g.append('rect') // outcome rect
      .attr('stroke', 'black')
      .attr('stroke-width', 1)
      .attr('fill', (d) => (d.active ? 'slateblue' : 'skyblue'))
      .attr('width', (d) => d.width)
      .attr('height', (d) => d.height)
      .attr('x', (d) => d.x - d.width / 2)
      .attr('y', (d) => d.y - d.height / 2);
    g.append('text') // outcome text
      .text((d) => d.name)
      .attr('font-weight', 'bold')
      .attr('font-size', 14)
      .attr('font-family', 'Times,serif')
      .attr('text-anchor', 'middle')
      .attr('dominant-baseline', 'middle')
      .attr('x', (d) => d.x)
      .attr('y', (d) => d.y);
  }

  return {
    modelsUpdated: modelsUpdated
  };
}
