# Isolated environment for gtsam_georeference.py: gtsam==4.2.1 requires
# numpy<2.0, which conflicts with the main worker image's numpy>=2/scipy/
# opencv stack -- see that script's module docstring. Kept as its own tiny
# image (no ROS, unlike worker-fastlio) rather than mixed into worker/.
FROM python:3.12-slim

RUN pip install --no-cache-dir gtsam==4.2.1 "numpy<2.0" laspy lazrs pyproj

COPY gtsam_georeference.py /srv/gtsam_georeference.py

ENTRYPOINT ["python3", "/srv/gtsam_georeference.py"]
