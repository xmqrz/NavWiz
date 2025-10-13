from __future__ import absolute_import
from __future__ import unicode_literals

from django.http import HttpResponse
from django.views.generic import View
from robohash import Robohash


class IndexView(View):
    # Generate the hash icon, given a seed string as input.

    def get(self, request, string=None, *args, **kwargs):
        # Set default values
        sizex = 300
        sizey = 300
        format = "png"
        bgset = None
        color = None

        # Ensure we have something to hash!
        if string is None:
            string = request.path

        # Split the size variable into sizex and sizey
        try:
            sizex, sizey = request.GET['size'].split('x')
            sizex = int(sizex)
            sizey = int(sizey)
        except Exception:
            pass
        finally:
            if sizex > 4096 or sizex <= 0:
                sizex = 300
            if sizey > 4096 or sizey <= 0:
                sizey = 300

        # Create our Robohashing object
        r = Robohash(string)

        # Allow users to manually specify a robot 'set' that they like.
        # Ensure that this is one of the allowed choices, or allow all
        # If they don't set one, take the first entry from sets above.

        if request.GET.get('set', r.sets[0]) in r.sets:
            roboset = request.GET.get('set', r.sets[0])
        elif request.GET.get('set', r.sets[0]) == 'any':
            roboset = r.sets[r.hasharray[1] % len(r.sets)]
        else:
            roboset = r.sets[0]

        # Only set1 is setup to be color-seletable. The others don't have enough pieces in various colors.
        # This could/should probably be expanded at some point..
        if request.GET.get('color') in r.colors:
            roboset = 'set1'
            color = request.GET.get('color')

        # If they DID choose set1, randomly choose a color.
        if roboset == 'set1' and color is None:
            color = r.colors[r.hasharray[0] % len(r.colors)]
            roboset = 'set1'

        # Allow them to set a background, or keep as None
        if request.GET.get('bgset') in r.bgsets + ['any']:
            bgset = request.GET.get('bgset')

        # Build our Robot.
        r.assemble(roboset=roboset, format=format, bgset=bgset, color=color, sizex=sizex, sizey=sizey)

        # Return the response
        response = HttpResponse(content_type='image/' + format)
        response['Cache-Control'] = 'public,max-age=31536000'
        r.img.save(response, format=r.format)
        return response
