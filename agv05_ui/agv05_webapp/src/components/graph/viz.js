import scene from './scene';

export default function (viz, data, opts) {
  /* Metadata */
  viz.meta = {
    width: 0,
    height: 0,
    marginBottom: 30,
    marginTop: 30,
    marginLeft: opts.marginLeft,
    marginRight: 30,
    legendMargin: 10,
    colorList: ['#ff7f0e', '#2ca02c', 'blue', 'red', 'purple', 'brown']
  };

  viz.data = data;

  viz.scene = scene(viz);

  viz.resize = function () {
    const bbox = viz.node().getBoundingClientRect();
    viz.meta.width = bbox.width;
    viz.meta.height = bbox.height;
    viz.scene.updated();
  };

  viz.theme = function () {
    viz.scene.updated();
  };

  function handleResize() {
    viz.resize();
  }
  addEventListener('resize', handleResize);

  const target = document.querySelector('html');
  let prevState = target.classList.contains('dark');
  const observer = new MutationObserver((mutations) => {
    mutations.forEach((mutation) => {
      if (mutation.attributeName === 'class') {
        const currentState = mutation.target.classList.contains('dark');
        if (prevState !== currentState) {
          prevState = currentState;
          viz.theme();
        }
      }
    });
  });
  observer.observe(target, {
    attributes: true,
    attributeFilter: ['style', 'class']
  });

  viz.destroy = function () {
    observer.disconnect();
    removeEventListener('resize', handleResize);
  };

  setTimeout(() => {
    viz.resize();
  }, 0);
  return viz;
}
