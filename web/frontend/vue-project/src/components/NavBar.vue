<script setup>
  import BellIcon from "./BellIcon.vue"


  //TODO: parse the json data comes from the sse and show them
  //then add them to an array each with an ID of its own
  const eventSource = new EventSource("http://127.0.0.1:5000/events?stream=messages")
  eventSource.onmessage = (event) => {
    console.log(event.data)
  }

</script>

<template>
  <nav class="bg-slate-100 w-screen sticky top-0 left-0 py-2 px-2">
    <ul class="flex justify-between">
      <h1 class="text-xl"><a href="/">Emergency System</a></h1>
      <BellIcon :count=0 :carData="[{
        carUUID: 'hehehehehe',
        langitude: 2.33,
        latitude: 4.333
      },
      {
        carUUID: 'hahaha',
        langitude: 2.33,
        latitude: 4.333
      }
      ]"/>
    </ul>
  </nav>
</template>


<style scoped>
</style>
