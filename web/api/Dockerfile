FROM node:24-bookworm-slim
WORKDIR /app
COPY api/server.mjs ./server.mjs
ENV RUBI_DB_PATH=/data/responses.sqlite RUBI_API_PORT=8787
EXPOSE 8787
CMD ["node", "server.mjs"]
