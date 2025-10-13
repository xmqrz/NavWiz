<script>
  import { mapChannel } from 'stores/sock/index.js';
  import mapViz from '$lib/shared/map/viz';

  var element;
  var viz;

  function vizInit(dom) {
    element = dom;
    viz = mapViz(dom, {
      grid: true,
      mapLayout: true
    });
    viz.zoom.zoomFit();
    mapChannel.subscribe(mapCallback);
    window.addEventListener('resize', vizResize);
    vizResize();

    setTimeout(function () {
      mapChannel.publish({
        id: 'retrieve_all'
      });
    }, 100);

    return {
      destroy() {
        mapChannel.unsubscribe(mapCallback);
        window.removeEventListener('resize', vizResize);
      }
    };
  }

  function vizResize() {
    var p = element.parentNode;
    var rc = p.getBoundingClientRect();
    if (viz) {
      viz.resized(rc.width, rc.height);
    }
  }

  function mapCallback(data) {
    if (data.id === 'robot_svg') {
      viz.scene.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'robot_pose') {
      viz.scene.updateRobotPose(data.robot_pose);
    } else if (data.id === 'robot_motion') {
      viz.scene.updateRobotMotion(data.robot_motion);
    } else if (data.id === 'map_layout') {
      try {
        viz.models.load(JSON.parse(data.map_layout));
      } catch (e) {
        console.log('Fail to parse map layout.');
      }
    }
  }
</script>

<div class="h-full w-full">
  <svg
    style="width: 100%; height: 100%;border-radius: 1rem"
    xmlns="http://www.w3.org/2000/svg"
    id="map-live-view"
    use:vizInit></svg>
</div>

<style global>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
