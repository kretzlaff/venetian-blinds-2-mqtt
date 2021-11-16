FROM python:3.10-slim-buster

COPY . /app
WORKDIR /app

RUN apt-get update && apt-get install gcc -y

RUN pip install pipenv --no-cache-dir
RUN pipenv install --system --deploy
RUN pip uninstall -y pipenv virtualenv-clone virtualenv

CMD ["python", "mqttblinds.py"]
