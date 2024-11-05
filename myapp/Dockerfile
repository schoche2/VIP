# Dockerfile
FROM node:18

# Create app directory
WORKDIR /usr/src/app

# Install app dependencies
COPY app/package*.json ./
RUN npm install

# Copy the app source code
COPY app .

# Expose the port that the app runs on
EXPOSE 3000

# Start the app
CMD ["node", "index.js"]
