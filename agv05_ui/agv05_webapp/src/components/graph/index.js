import * as d3 from 'd3';
import Viz from './viz';

export function graph(dom, data) {
  const svg = d3.select(dom).append('svg');
  svg.attr('class', 'w-full h-full ' + data.graphClass);
  const viz = Viz(svg, data.data, {
    marginLeft: data.marginLeft
  });

  return {
    update() {},
    destroy() {
      viz.destroy();
    }
  };
}
