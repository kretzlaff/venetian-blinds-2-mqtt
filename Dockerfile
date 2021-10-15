FROM arm32v7/python:3.10-slim

COPY . /app
WORKDIR /app

RUN pip install pipenv --no-cache-dir && \
    pipenv install --system --deploy && \
    pip uninstall -y pipenv virtualenv-clone virtualenv

CMD ["python", "mqttblinds.py"]
